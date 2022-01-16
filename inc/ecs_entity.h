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
using EntityPtr = Entity *;
using EntityConstPtr = Entity const *;

class ComponentBase;
using ComponentPtr = std::shared_ptr<ComponentBase>;


class Entity
{
public:
    Entity(const std::string& name)
    :name_(name), parent_(nullptr)
    {
    }
    std::map<ComponentType , ComponentPtr>& components() { return components_; }
    const std::map<ComponentType , ComponentPtr>& components() const { return components_; }

    // std::map<std::string, EntityPtr> & children() {return children_;}
    void addChild(EntityPtr child)
    {
        children_[child->name_] = child;
        child->setParent(this);
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

    EntityPtr root() {
        EntityPtr tmp_ptr = this;
        for(size_t i = 0; i < 99; i++)
        {
            if(nullptr == tmp_ptr->parent_) return tmp_ptr;
            tmp_ptr = tmp_ptr->parent_;
        }
        return nullptr;
    }

    bool isLeaf() const { return children_.size() == 0; }

    const std::map<std::string, EntityPtr>& children() const { return children_; }
    std::map<std::string, EntityPtr>& children() { return children_; }

    EntityPtr parent() { return parent_; }
    const EntityPtr parent() const { return parent_; }

protected:
    void setParent(EntityPtr parent)
    {
        parent_ = parent;
    }
    std::string name_;
    EntityPtr parent_;
    std::map<std::string, EntityPtr> children_;
    std::map<ComponentType , ComponentPtr> components_;
};


} // namespace ecs

} // namespace rtc


#endif // _ECS_ENTITY_H_
