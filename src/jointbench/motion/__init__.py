"""Motion machinery for JointBench: mechanisms whose parts move in parallel
planes, graded as a series of still positions.

    from jointbench.motion import generate, grade
    task, mech, reference = generate.make("fourbar", seed=0)
    report = grade.grade_solid(task, hidden_mechanism, reference)

See docs/motion.md. Run as ``python -m jointbench.motion``.
"""
