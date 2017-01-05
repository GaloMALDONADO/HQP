from setuptools import setup, find_packages

setup(
    name='hqp',
    description='Hierarchichal Quadratic Programming',
    version='1.0',

    packages=find_packages(exclude=['data','tests','examples']),
    
    author='Galo MALDONADO',
    author_email='galo.maldonado@laas.fr',
    
    #install_requires = [
    #    "pinocchio"
    #],

    classifiers=[
        'Intended Audience :: Science/Research',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
    ],
)
