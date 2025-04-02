#pragma once

#include <functional>
#include <map>

#include "state/state_base.h"
#include "state/state_type.h"

namespace CubliMini {
namespace State {

class StateFactory
{
   private:
    using Creator = std::function<StateBase *()>;
    std::map<StateType, Creator> creators;

   public:
    template<typename T>
    void registerState(StateType type)
    {
        creators[type] = []() { return new T; };
    }

    StateBase *create(StateType type)
    {
        auto it = creators.find(type);
        return (it != creators.end()) ? it->second() : nullptr;
    }

    static StateFactory &instance()
    {
        static StateFactory factory;
        return factory;
    }
};
}  // namespace State
}  // namespace CubliMini