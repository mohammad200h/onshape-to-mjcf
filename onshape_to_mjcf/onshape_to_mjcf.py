from .mjcf.util import get_assembly
from .mjcf.model import create_model
from .onshape_api.client import Client
from .onshape_api.config import config, configFile


def main():
  # OnShape API client
  workspaceId = None
  client = Client(logging=False, creds=configFile)
  client.useCollisionsConfigurations = config["useCollisionsConfigurations"]

  assembly = get_assembly(client)

  create_model(client,assembly)

if __name__ == "__main__":
  main()
