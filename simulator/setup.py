from setuptools import setup, find_packages

setup(name='simulator',
      version='0.0.1',
      author='Soham Lakhi',
      author_email='sohamlakhi39@gmail.com',
      packages=find_packages(),
      install_requires=['f110-gym==0.2.1', 'matplotlib', 'pandas']
      )
      
