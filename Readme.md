# Getting Started
This package is inspired by [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot). It supports closed-loop kinematics as well as assemblies with multiple subassemblies. However, nested assemblies are not supported yet. An example of closed-loop kinematics can be found in the examples folder. The package is actively under development. Please report any issues or suggestions :D
## Installation
You can Install the package using the following commad:
 ```
 sudo pip install .
 ```
## Using the package
To use the pakage you will need an `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
Follow instruction in [here](https://www.onshape.com/en/features/integrations)  to get `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
Once you have them create a shell script to make sourcing environment variables easier.
create a file onshapeKey.sh with the following content

```
#onshapeKey.sh
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=<your_access_key>
export ONSHAPE_SECRET_KEY=<your_secret_key>
```

Source the environment variables.

```
source onshapeKey.sh
```

Now you can start using the package. onshape-to-mjcf  needs a config file with information indicating where it can find the assembly file.
Next lets create config.json file and place it in  a empty directory called iiwa14. We will be using publicly available iiwa14 assembly: https://cad.onshape.com/documents/b032c973e321f949e1feb872/w/92fa339d0e4305c61596a06d/e/cafb200fbd5b69c229bbb366.

```
#config.json

{
    "documentId":"b032c973e321f949e1feb872",
    "outputFormat":"urdf",
    "packageName":"iiwa_14",
    "robotName":"iiwa14",
    "assemblyName":"IIwa14_Assembly"
}
```
if you look at the link you can find the `documentId` after `https://cad.onshape.com/documents/`
Also if you follow the link you will see that `assemblyName` Matches the name given to the assembly. Onshape Api uses `documentId` and `assemblyName` to find the assembly.

Now, we can create the MJCF file using the following command inside the directory where config file is:

```
cd iiwa14
onshape-to-mjcf .
```


## Video Tutorials
Here you can see three different video tutorials demonstrating a simple tree, closed-loop kinematics, and combining exported models to make a new model.

### Simple Robot
[![Watch the video](https://img.youtube.com/vi/N9xQnkLXntE/hqdefault.jpg)](https://www.youtube.com/watch?v=N9xQnkLXntE)

### Closed Loop Kinematics
[![Watch the video](https://img.youtube.com/vi/BbLfnp1ao00/hqdefault.jpg)](https://www.youtube.com/watch?v=BbLfnp1ao00)

### Creating a Model by Combining First Two Models
[![Watch the video](https://img.youtube.com/vi/3YtK7WJ2qSg/hqdefault.jpg)](https://www.youtube.com/watch?v=3YtK7WJ2qSg)





## Under development feature
We are working on Onshape's group feature. It will come out soon. It enable creating a body from group where all the elements within the group are treated as geoms belonging to the body.
