/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <moveit_setup_framework/utilities.hpp>

namespace moveit_setup
{
bool extractPackageNameFromPath(const std::filesystem::path& path, std::string& package_name,
                                std::filesystem::path& relative_filepath)
{
  std::filesystem::path sub_path = path;  // holds the directory less one folder
  if (std::filesystem::is_regular_file(sub_path))
  {
    relative_filepath = sub_path.filename();  // holds the path after the sub_path
    sub_path = sub_path.parent_path();
  }
  else
  {
    relative_filepath = sub_path;
  }

  // truncate path step by step and check if it contains a package.xml
  // This runs until the path is either empty "" or at the root "/" or "C:\\"
  while (!sub_path.empty() && sub_path != sub_path.root_path())
  {
    if (std::filesystem::is_regular_file(sub_path / "package.xml"))
    {
      // Search the <name> </name> string in the package.xml file
      // This works assuming the package name is entered as <name>PACKAGE_NAME</name>
      // Default package name to folder name
      package_name = sub_path.filename().string();
      tinyxml2::XMLDocument package_xml_file;
      auto is_open = package_xml_file.LoadFile((sub_path / "package.xml").c_str());
      if (is_open == tinyxml2::XML_SUCCESS)
      {
        auto name_potential =
            package_xml_file.FirstChildElement("package")->FirstChildElement("name")->FirstChild()->ToText()->Value();
        if (name_potential)
        {
          // Change package name if we have non-empty potential, else it defaults
          package_name = name_potential;
        }
      }
      return true;
    }
    relative_filepath = sub_path.filename() / relative_filepath;
    sub_path = sub_path.parent_path();
  }
  // No package name found, we must be outside ROS
  return false;
}

bool hasRequiredAttributes(const tinyxml2::XMLElement& e, const std::vector<XMLAttribute>& attributes)
{
  for (const auto& attr : attributes)
  {
    if (!attr.required)
      continue;  // attribute not required
    const char* value = e.Attribute(attr.name);
    if (value && strcmp(attr.value, value) == 0)
    {
      continue;  // attribute has required value
    }
    else
    {
      return false;
    }
  }
  return true;
};

tinyxml2::XMLElement* uniqueInsert(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement& element, const char* tag,
                                   const std::vector<XMLAttribute>& attributes, const char* text)
{
  // search for existing element with required tag name and attributes
  tinyxml2::XMLElement* result = element.FirstChildElement(tag);
  while (result && !hasRequiredAttributes(*result, attributes))
    result = result->NextSiblingElement(tag);

  if (!result)  // if not yet present, create new element
  {
    result = doc.NewElement(tag);
    element.InsertEndChild(result);
  }

  // set (not-yet existing) attributes
  for (const auto& attr : attributes)
  {
    if (!result->Attribute(attr.name))
      result->SetAttribute(attr.name, attr.value);
  }

  // insert text if required
  if (text && !result->GetText())
  {
    tinyxml2::XMLText* text_el = doc.NewText(text);
    result->InsertEndChild(text_el);
  }

  return result;
}

}  // namespace moveit_setup
