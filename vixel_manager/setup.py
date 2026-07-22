from setuptools import find_packages, setup


package_name = "vixel_manager"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="sn3rt",
    maintainer_email="sn3rt@users.noreply.github.com",
    description="Inventory and orchestration manager for Vixel sensors.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "inventory_manager = vixel_manager.manager_node:main",
            "vixel = vixel_manager.cli:main",
        ],
    },
)
