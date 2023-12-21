#include <sofa/core/ObjectFactory.h>
#include <InfinyToolkit/MyComponent.h>

namespace sofa::infinytoolkit
{

   int MyComponentClass = sofa::core::RegisterObject("This is my basic component that does nothing!")
        .add< MyComponent>();
  

} // namespace sofa::infinytoolkit