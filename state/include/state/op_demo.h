#ifndef OP_DEMO_H_
#define OP_DEMO_H_

namespace robotis_op
{

class Dance
{
 public:
  enum Motion_Index
  {
    InitPose = 144,
    WalkingReady = 144,
    GetUpFront = 122,
    GetUpBack = 123,
    RightKick = 121,
    LeftKick = 120,
    Ceremony = 27,
    ForGrass = 20,
  };

  Dance()
  {
  }
  virtual ~Dance()
  {
  }

  virtual void setDemoEnable()
  {
  }
  virtual void setDemoDisable()
  {
  }

 protected:
  bool enable_;
};

} /* namespace robotis_op */

#endif /* OP_DEMO_H_ */