from setuptools import find_packages, setup

package_name = "vlm_perception"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/perception.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="WEDA industrial VLM reference app: description + OCR + depth grounding.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vlm-perception = vlm_perception.app:main",
            "vlm-depth = vlm_perception.depth_processor:main",
            "vlm-serve = vlm_perception.serve_vlm:main",
            "vlm-feed = vlm_perception.feed:main",
            "vlm-view = vlm_perception.view:main",
            "vlm-detector = vlm_perception.detector_node:main",
        ],
    },
)
