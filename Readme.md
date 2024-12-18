# Getting Started:

## Installation
You can Install the package using the following commad:
 ```
 sudo pip install .
 ```
## Using the package

## setup
To use the pakage you will need an `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
Follow instruction in https://www.onshape.com/en/features/integrations to get `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
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
Next lets create config.json file and place it in  a empty directory called iiwa14. We will be using publicly available iiwa14 assembly https://cad.onshape.com/documents/b032c973e321f949e1feb872/w/92fa339d0e4305c61596a06d/e/cafb200fbd5b69c229bbb366.

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

Now we can create the MJCF file using the following command inside the directory where config file is:

```
cd iiwa14
onshape-to-mjcf .
```


