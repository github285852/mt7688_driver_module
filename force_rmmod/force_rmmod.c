#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <linux/list.h>
#include <asm-generic/local.h>
#include <linux/platform_device.h>
#include <linux/kallsyms.h>
#include <linux/sched.h>

int main(int argc, char **argv)
{
    struct module   *mod = NULL, *relate = NULL;
    int              cpu;
#ifdef CONFIG_REPLACE_EXIT_FUNCTION
    void            *origin_exit_addr = NULL;
#endif
	if(argc<2)
	{
		printf("enter force rmmod name\n");
		return 0;
	}
    /////////////////////
    //  找到待删除模块的内核module信息
    /////////////////////
#if 0
    //  方法一, 遍历内核模块树list_mod查询
    struct module *list_mod = NULL;
    /*  遍历模块列表, 查找 del_mod_name 模块  */
    list_for_each_entry(list_mod, THIS_MODULE->list.prev, list)
    {
        if (strcmp(list_mod->name, del_mod_name) == 0)
        {
            mod = list_mod;
        }
    }
    /*  如果未找到 del_mod_name 则直接退出  */
    if(mod == NULL)
    {
        printk("[%s] module %s not found\n", THIS_MODULE->name, modname);
        return -1;
    }
#endif

    //  方法二, 通过find_mod函数查找
    if((mod = find_module(argv[2])) == NULL)
    {
        printk("[%s] module %s not found\n", THIS_MODULE->name, del_mod_name);
        return -1;
    }
    else
    {
        printk("[before] name:%s, state:%d, refcnt:%u\n",
                mod->name ,mod->state, module_refcount(mod));
    }

    /////////////////////
    //  如果有其他驱动依赖于当前驱动, 则不能强制卸载, 立刻退出
    /////////////////////
    /*  如果有其他模块依赖于 del_mod  */
    if (!list_empty(&mod->source_list))
    {
        /*  打印出所有依赖target的模块名  */
        list_for_each_entry(relate, &mod->source_list, source_list)
        {
            printk("[relate]:%s\n", relate->name);
        }
    }
    else
    {
        printk("No modules depond on %s...\n", del_mod_name);
    }

    /////////////////////
    //  清除驱动的状态和引用计数
    /////////////////////
    //  修正驱动的状态为LIVE
    mod->state = MODULE_STATE_LIVE;

    //  清除驱动的引用计数
    for_each_possible_cpu(cpu)
    {
        local_set((local_t*)per_cpu_ptr(&(mod->refcnt), cpu), 0);
        //local_set(__module_ref_addr(mod, cpu), 0);
        //per_cpu_ptr(mod->refptr, cpu)->decs;
        //module_put(mod);
    }
    atomic_set(&mod->refcnt, 1);

#ifdef CONFIG_REPLACE_EXIT_FUNCTION
    /////////////////////
    //  重新注册驱动的exit函数
    /////////////////////
    origin_exit_addr = mod->exit;
    if (origin_exit_addr == NULL)
    {
        printk("module %s don't have exit function...\n", mod->name);
    }
    else
    {
        printk("module %s exit function address %p\n", mod->name, origin_exit_addr);
    }

    mod->exit = force_replace_exit_module_function;
    printk("replace module %s exit function address (%p -=> %p)\n", mod->name, origin_exit_addr, mod->exit);
#endif

    printk("[after] name:%s, state:%d, refcnt:%u\n",
            mod->name, mod->state, module_refcount(mod));

    return 0;
}



