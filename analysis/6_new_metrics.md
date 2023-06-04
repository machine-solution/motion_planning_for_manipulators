# New metrics

## Changes in calculating cost of move
Before cost of move `a` was equal `abs(a)`, now cost of move `a` *after move `b`* is equal `abs(a) + abs(a-b) * const`. It corresponds to the desire to find more smooth path i.e. without a lot of changes of type of move. If we can move first joint 10 times and after move second joint 10 times we don't want to move joints in order "first, second, first, second, ...".
This const is `g_weightSmoothness` defined in "global_defs.h".
