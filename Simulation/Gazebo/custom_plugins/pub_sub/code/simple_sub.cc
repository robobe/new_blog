#include <gz/common/Console.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <mutex>
#include <string>

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class SimpleSub
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate
{
public:
  void Configure(const Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &,
                 EventManager &) override
  {
    this->topic = "/simple_sub";
    if (_sdf && _sdf->HasElement("topic"))
      this->topic = _sdf->Get<std::string>("topic");

    if (!this->node.Subscribe(this->topic, &SimpleSub::OnMsg, this))
    {
      gzerr << "[SimpleSub] Failed to subscribe to [" << this->topic << "]"
            << std::endl;
    }
    else
    {
      gzmsg << "[SimpleSub] Subscribed to [" << this->topic << "]" << std::endl;
    }
  }

  void PostUpdate(const UpdateInfo &,
                  const EntityComponentManager &) override
  {
    std::string msg;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->pending.empty())
        return;
      msg = std::move(this->pending);
      this->pending.clear();
    }

    gzmsg << "[SimpleSub] " << msg << std::endl;
  }

private:
  void OnMsg(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pending = _msg.data();
  }

  transport::Node node;
  std::mutex mutex;
  std::string pending;
  std::string topic;
};

GZ_ADD_PLUGIN(SimpleSub,
              System,
              SimpleSub::ISystemConfigure,
              SimpleSub::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(SimpleSub,
                    "gz::sim::systems::SimpleSub")
