#if !defined(_ECS_ENTITY_H_)
#define _ECS_ENTITY_H_

#include <vector>
#include <map>
#include <string>
#include <memory>

#include "ecs_component.h"

namespace rtc
{
namespace ecs
{

class Entity;
using EntityPtr = Entity*;

class ComponentBase;
using ComponentPtr = std::shared_ptr<ComponentBase>;


class Entity
{
public:
    Entity(const std::string& name)
    :name_(name), root_(this)
    {
    }
    std::map<ComponentType , ComponentPtr>& components(ComponentType target_type) { return components_; }
    const std::map<ComponentType , ComponentPtr>& components(ComponentType target_type) const { return components_; }

    // std::map<std::string, EntityPtr> & children() {return children_;}
    void addChild(EntityPtr child)
    {
        children_[child->name_] = child;
        child->setRoot(root_);
    }

    void removeChild(const std::string& name)
    {
        if(children_.count(name) == 0) return;
        children_.erase(name);
    }

    EntityPtr findChild(const std::string& name)
    {
        if(children_.count(name) > 0) return children_[name];
        for(const auto & name_entity_pair: children_)
        {
            auto grand_child = name_entity_pair.second->findChild(name);
            if(grand_child != nullptr)
                return grand_child;
        }
        return nullptr;
    }

    EntityPtr root() const { return root_; }

protected:
    void setRoot(EntityPtr root)
    {
        root_ = root;
        for(auto & name_entity_pair: children_)
            name_entity_pair.second->setRoot(root);
    }
    std::string name_;
    EntityPtr root_;
    std::map<std::string, EntityPtr> children_;
    std::map<ComponentType , ComponentPtr> components_;
};

} // namespace ecs

} // namespace rtc


#endif // _ECS_ENTITY_H_
