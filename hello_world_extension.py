import os

from omni.isaac.examples.base_sample import BaseSampleExtension

from omni.isaac.examples.user_examples import CustomWorld



class CustomWorldExtension(BaseSampleExtension):

    def on_startup(self, ext_id: str):

        super().on_startup(ext_id)

        super().start_extension(

            menu_name="",

            submenu_name="",

            name="Awesome Example",

            title="My Awesome Example",

            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html",

            overview="This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",

            file_path=os.path.abspath(__file__),

            sample=CustomWorld(),

        )

        return
    