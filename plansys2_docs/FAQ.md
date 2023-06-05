---
tip: translate by baidu@2023-06-05 13:59:59
...

# FAQ

## 1. How to debug errors in PDDL?

Working with PDDL is difficult. If the PDDL domain is not well designed, or predicates or instances are missing, it is impossible to generate a plan. Plansys2 will notify you, but it is difficult to debug and solve the problem.

> 使用 PDDL 很困难。如果 PDDL 域设计不好，或者谓词或实例丢失，则无法生成计划。Plansys2 会通知您，但很难调试和解决问题。

When it is required to generate a plan, Plansys2 generates a file with the domain, `/tmp/${namespace}/domain.pddl` and another with the problem `/tmp/${namespace}/domain.pddl`. In addition, the output of the plan solver (popf) is saved in /tmp/${namespace}/plan.pddl.

> 当需要生成计划时，Plansys2 会生成一个域为“/tmp/${namespace}/domain.pddl”的文件和另一个问题为“/tmp/${namespace}/domain.pdf”的文件。此外，计划求解器（popf）的输出保存在/tmp/${namespace}/plan.pddl 中。

It is possible to execute the plan solver in isolation using the command:

> 可以使用以下命令单独执行计划求解器：

```shell
ros2 run popf popf /tmp/${namespace}/domain.pddl /tmp/${namespace}/problem.pddl
```

Use `ros2 run popf popf -h` for more help.

> 使用“ros2 run popf popf-h”可获得更多帮助。
