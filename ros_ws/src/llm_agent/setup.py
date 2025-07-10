from setuptools import find_packages, setup

package_name = "llm_agent"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "openai>=1.0.0",
        "google-generativeai>=0.3.0",
        "pydantic>=2.0.0",
        "python-dotenv>=1.0.0",
        "requests>=2.31.0",
    ],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@todo.todo",
    description="Interfaces with cloud LLMs to convert natural language to structured JSON mission plans",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "llm_agent_node = llm_agent.node:main",
        ],
    },
)
