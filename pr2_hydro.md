# How to use the PR2 with Hydro

There are a couple of bugs with PR2 Hydro. Until the packages get updated, the following steps are necessary.

## Tucking the arms
In the Hydro version of pr2_tuck_arms_action, tuck_arms.py only starts the action server, but doesn't send a goal to tuck the arms.

For now, either write a SimpleActionClient that tucks the arms yourself, or get a different version of pr2_common_actions [(jstnhuang/pr2_common_actions)](https://github.com/jstnhuang/pr2_common_actions).
