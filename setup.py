from setuptools import find_packages, setup

PACKAGE_NAME = "g1_robot_sim"

# Read dependencies from requirements.txt file
with open('requirements.txt', 'r') as f:
    install_deps = [line.strip() for line in f.readlines() if line.strip()]

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=find_packages(where="src"),  # Automatically find packages in the "src" directory
    package_dir={"": "src"},             # Define "src" as the root for packages
    include_package_data=True,           # Include additional data specified in MANIFEST.in
    zip_safe=False,      # Ensure the package can be installed in non-zip-safe environments
    install_requires=install_deps,
    setup_requires=[
        # List setup-specific dependencies here if needed
    ],
    python_requires=">=3.10",
    package_data={
        PACKAGE_NAME: [f'src/{PACKAGE_NAME}/graph_config/*.yml'],
    },
    long_description=long_description,
    long_description_content_type="text/markdown",
)
