class OldAPI:
    def specific_request(self):
        return "Old API response"
    
# the client call requires a different interface
# we want to use OldAPI but it doesn't match the expected interface
# the Adapter allows us to use OldAPI with the new interface
# the client code can call Adapter.request() and it will internally call OldAPI.specific_request()

class Adapter:
    def __init__(self, old_api):
        self.old_api = old_api

    def request(self):
        return self.old_api.specific_request()
    
# client code
old_api = OldAPI()
adapter = Adapter(old_api)

print(adapter.request())