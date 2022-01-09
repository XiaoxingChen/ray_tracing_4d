#if !defined(_ECS_COMPONENT_H_)
#define _ECS_COMPONENT_H_


namespace rtc
{
namespace ecs
{

enum ComponentType
{
    // eUnknown = 0,
    eMesh = 1,
    eTransform = 2,
    eMaterial = 3
};

class ComponentBase
{
protected:
    /* data */
public:
    ComponentBase(/* args */){};
    ~ComponentBase(){};
    virtual ComponentType type() = 0;
};




} // namespace ecs

} // namespace rtc



#endif // _ECS_COMPONENT_H_
