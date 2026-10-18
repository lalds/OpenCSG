#include "realtimecsg/recovered_scene.h"
#include <array>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace {
using namespace realtimecsg::recovered;
std::mutex registryMutex;
std::map<std::uint64_t,std::shared_ptr<const ExportedScene>> registry;
std::uint64_t nextHandle=1;
thread_local char lastError[512]{};
template<class F> int call(F operation) noexcept {
    lastError[0]=0;
    try { operation(); return 0; }
    catch (const std::invalid_argument& e) { std::snprintf(lastError,sizeof(lastError),"%s",e.what()); return -1; }
    catch (const std::length_error& e) { std::snprintf(lastError,sizeof(lastError),"%s",e.what()); return -2; }
    catch (const std::exception& e) { std::snprintf(lastError,sizeof(lastError),"%s",e.what()); return -3; }
    catch (...) { std::snprintf(lastError,sizeof(lastError),"Unknown scene bridge error"); return -3; }
}
std::shared_ptr<const ExportedScene> snapshot(std::uint64_t handle) {
    std::lock_guard<std::mutex> lock(registryMutex); const auto found=registry.find(handle);
    if (found==registry.end()) throw std::invalid_argument("Invalid or released scene handle");
    return found->second;
}
struct Range { std::uintptr_t begin,end; };
template<class T> Range checked(T* data,int capacity,std::size_t count) {
    if (capacity<0 || (capacity&&!data)) throw std::invalid_argument("Invalid scene output buffer");
    if (static_cast<std::size_t>(capacity)<count) throw std::length_error("Scene output capacity exhausted");
    const auto begin=reinterpret_cast<std::uintptr_t>(data);
    if (count>(std::numeric_limits<std::uintptr_t>::max()-begin)/sizeof(T)) throw std::invalid_argument("Scene output range overflow");
    return {begin,begin+count*sizeof(T)};
}
template<class T> void copy(T* destination,const std::vector<T>& data) {
    if (!data.empty()) std::memcpy(destination,data.data(),data.size()*sizeof(T));
}
}
extern "C" std::uint32_t rcsg_recovered_scene_abi_version() { return realtimecsg::recovered::kSceneAbiVersion; }
extern "C" const char* rcsg_recovered_scene_last_error() { return lastError; }
extern "C" std::int32_t rcsg_recovered_scene_build(const realtimecsg::recovered::SceneBuildInput* input,std::uint64_t* handle) {
    return call([&]() {
        if (!input||!handle) throw std::invalid_argument("Missing scene input/handle");
        auto scene=std::make_shared<const realtimecsg::recovered::ExportedScene>(realtimecsg::recovered::buildConvexScene(*input));
        std::lock_guard<std::mutex> lock(registryMutex);
        if (!nextHandle) throw std::length_error("Scene handle space exhausted");
        const auto id=nextHandle; registry.emplace(id,std::move(scene)); ++nextHandle; *handle=id;
    });
}
extern "C" std::int32_t rcsg_recovered_scene_counts(std::uint64_t handle,realtimecsg::recovered::SceneCounts* counts) {
    return call([&]() { if (!counts) throw std::invalid_argument("Missing scene counts"); *counts=snapshot(handle)->counts; });
}
extern "C" std::int32_t rcsg_recovered_scene_copy(std::uint64_t handle,const realtimecsg::recovered::SceneOutputBuffers* output) {
    return call([&]() {
        if (!output) throw std::invalid_argument("Missing scene output descriptor");
        const auto scene=snapshot(handle); const auto buffers=*output;
        const std::array<Range,4> ranges{{checked(buffers.vertices,buffers.vertexCapacity,scene->vertices.size()),
            checked(buffers.indices,buffers.indexCapacity,scene->indices.size()),checked(buffers.submeshes,buffers.submeshCapacity,scene->submeshes.size()),
            checked(buffers.triangles,buffers.triangleCapacity,scene->triangles.size())}};
        for (std::size_t i=0;i<ranges.size();++i) for (std::size_t j=i+1;j<ranges.size();++j)
            if (ranges[i].begin<ranges[i].end && ranges[j].begin<ranges[j].end && ranges[i].begin<ranges[j].end && ranges[j].begin<ranges[i].end)
                throw std::invalid_argument("Scene output buffers overlap");
        copy(buffers.vertices,scene->vertices); copy(buffers.indices,scene->indices);
        copy(buffers.submeshes,scene->submeshes); copy(buffers.triangles,scene->triangles);
    });
}
extern "C" std::int32_t rcsg_recovered_scene_release(std::uint64_t handle) {
    return call([&]() { std::lock_guard<std::mutex> lock(registryMutex); if (!registry.erase(handle)) throw std::invalid_argument("Invalid or released scene handle"); });
}
