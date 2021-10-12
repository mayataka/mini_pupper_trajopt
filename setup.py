from setuptools import find_packages, setup

setup(
    name='mini_pupper_trajopt',
    version='0.0.1',
    description="TODO",
    author='Sotaro Katayama',
    author_email='katayama.25w@st.kyoto-u.ac.jp',
    platforms=['any'],
    license="MIT",
    url='https://github.com/mayataka/mini-pupper-trajopt',
    package_data={'': ['mini_pupper_description']},
    include_package_data=True,
    packages=find_packages(),
    install_requires=['numpy']
    # install_requires=['numpy', 'meshcat-python']
)