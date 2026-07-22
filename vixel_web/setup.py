from setuptools import find_packages, setup


package_name = "vixel_web"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/static", ["vixel_web/static/index.html"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="sn3rt",
    maintainer_email="sn3rt@users.noreply.github.com",
    description="Loopback web management and compressed-image gateway for Vixel.",
    license="MIT",
    entry_points={
        "console_scripts": ["web_gateway = vixel_web.gateway:main"],
    },
)
