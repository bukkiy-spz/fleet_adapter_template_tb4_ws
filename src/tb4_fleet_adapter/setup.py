from setuptools import setup
from setuptools.command.develop import develop as _develop

package_name = 'tb4_fleet_adapter'


class develop(_develop):
    """Compatibility shim for colcon + newer setuptools."""

    user_options = _develop.user_options + [
        ('uninstall', None, 'Ignored colcon compatibility option'),
        ('editable', None, 'Ignored colcon compatibility option'),
        ('build-directory=', None, 'Ignored colcon compatibility option'),
    ]
    boolean_options = _develop.boolean_options + ['uninstall', 'editable']

    def initialize_options(self):
        super().initialize_options()
        self.uninstall = False
        self.editable = False
        self.build_directory = None

    def run(self):
        # colcon invokes `setup.py develop --uninstall --editable ...`.
        # Newer setuptools rejects these extra flags, so we absorb them here
        # and perform a normal develop install.
        self.uninstall = False
        return super().run()

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yadunund',
    maintainer_email='yadunund@openrobotics.org',
    description='A template for an RMF fleet adapter',
    license='Apache License 2.0',
    cmdclass={'develop': develop},
    entry_points={
        'console_scripts': [
            'fleet_adapter=tb4_fleet_adapter.fleet_adapter:main'
        ],
    },
)
