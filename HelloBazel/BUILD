load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_library(
	name = "test_lib",
	srcs = glob(["*.cpp"], exclude = ["Test.cpp"]),
    hdrs = glob(["*.hpp"]),
    copts = [""],
    deps = ["@opencv//:opencv"],
    visibility = ["//visibility:public"],
	linkstatic = 1,
	)

cc_binary(
	name = "test",
	srcs = ["Test.cpp"],
    deps = ["@opencv//:opencv", "//HelloBazel:test_lib"],
	)

    