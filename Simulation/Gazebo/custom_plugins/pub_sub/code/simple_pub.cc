#include <gz/common/Console.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <chrono>
#include <string>

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class SimplePub
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
    this->payload = "hello";
    this->rateHz = 1.0;

    if (_sdf)
    {
      if (_sdf->HasElement("topic"))
        this->topic = _sdf->Get<std::string>("topic");
      if (_sdf->HasElement("message"))
        this->payload = _sdf->Get<std::string>("message");
      if (_sdf->HasElement("rate_hz"))
        this->rateHz = _sdf->Get<double>("rate_hz");
    }

    if (this->rateHz > 0.0)
      this->period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / this->rateHz));
    else
      this->period = std::chrono::nanoseconds::zero();

    // Advertise the topic
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(this->topic);
    if (!this->publisher)
    {
      gzerr << "[SimplePub] Failed to advertise [" << this->topic << "]"
            << std::endl;
    }
    else
    {
      gzmsg << "[SimplePub] Publishing on [" << this->topic << "]"
            << std::endl;
    }

    this->msg.set_data(this->payload);
    this->lastPubTime = std::chrono::nanoseconds::min();
  }

  void PostUpdate(const UpdateInfo &_info,
                  const EntityComponentManager &) override
  {
    if (_info.paused || !this->publisher)
      return;

    if (this->period == std::chrono::nanoseconds::zero())
      return;

    if (this->lastPubTime == std::chrono::nanoseconds::min() ||
        _info.simTime - this->lastPubTime >= this->period)
    {
      this->lastPubTime = _info.simTime;
      // publish the message
      this->publisher.Publish(this->msg);
    }
  }

private:
  transport::Node node;
  transport::Node::Publisher publisher;
  std::string topic;
  std::string payload;
  double rateHz{1.0};
  std::chrono::nanoseconds period{0};
  std::chrono::nanoseconds lastPubTime{0};
  gz::msgs::StringMsg msg;
};

GZ_ADD_PLUGIN(SimplePub,
              System,
              SimplePub::ISystemConfigure,
              SimplePub::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(SimplePub,
                    "gz::sim::systems::SimplePub")
