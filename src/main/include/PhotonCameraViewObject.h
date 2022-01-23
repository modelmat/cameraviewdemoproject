// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <networktables/NetworkTable.h>
#include <wpi/StringMap.h>

#include <frc/Errors.h>

/**
 * Common base class for all Mechanism2d node types.
 *
 * To append another node, call Append with the type of node and its
 * construction parameters. None of the node types are designed to be
 * constructed directly, and are owned by their parent node/container - obtain
 * pointers from the Append function or similar factory methods.
 *
 * @see Mechanism2d.
 */
class PhotonCameraViewObject {
  friend class PhotonCameraView;

 protected:
  explicit PhotonCameraViewObject(std::string_view name);

  /**
   * Update all entries with new ones from a new table.
   *
   * @param table the new table.
   */
  virtual void UpdateEntries(std::shared_ptr<nt::NetworkTable> table) = 0;

  mutable wpi::mutex m_mutex;

 public:
  virtual ~PhotonCameraViewObject() = default;

  /**
   * Retrieve the object's name.
   *
   * @return the object's name relative to its parent.
   */
  const std::string& GetName() const;

  /**
   * Append a Mechanism object that is based on this one.
   *
   * @param name the name of the new object.
   * @param args constructor arguments of the object type.
   * @return the constructed and appended object, useful for variable
   * assignments and call chaining.
   * @throw if an object with the given name already exists.
   */
  template <typename T, typename... Args,
            typename =
                std::enable_if_t<std::is_convertible_v<T*, PhotonCameraViewObject*>>>
  T* Append(std::string_view name, Args&&... args) {
    std::scoped_lock lock(m_mutex);
    auto& obj = m_objects[name];
    if (obj) {
      throw FRC_MakeError(
          frc::err::Error,
          "PhotonCameraViewObject names must be unique! `{}` was inserted twice!",
          name);
    }
    obj = std::make_unique<T>(name, std::forward<Args>(args)...);
    T* ex = static_cast<T*>(obj.get());
    if (m_table) {
      ex->Update(m_table->GetSubTable(name));
    }
    return ex;
  }

 private:
  std::string m_name;
  wpi::StringMap<std::unique_ptr<PhotonCameraViewObject>> m_objects;
  std::shared_ptr<nt::NetworkTable> m_table;
  void Update(std::shared_ptr<nt::NetworkTable> table);
};
