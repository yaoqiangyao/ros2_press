# Copyright 2024 Apex.AI, Inc.
# All rights reserved.

def performance_test_with_plugin(name, plugin, **kwargs):
    native.cc_binary(
        name=name,
        srcs = ["@performance_test//performance_test:performance_test_main"],
        deps = [
            "@performance_test//performance_test:performance_test_lib",
            plugin,
        ],
        linkopts = select({
            "@platforms//os:linux": ["-lpthread"],
            "//conditions:default": [],
        }),
        visibility = ["//visibility:public"],
        **kwargs
    )
