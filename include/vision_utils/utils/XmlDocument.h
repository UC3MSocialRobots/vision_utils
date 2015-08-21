/*!
     * \file XmlDocument.h
 *
 * A class to ease the I/O of XML files
 *
 * \date Dec 19, 2010
 * \author Arnaud Ramey
 */
#ifndef XMLDOCUMENT_H_
#define XMLDOCUMENT_H_

#define XML_IMPLEMENTATION_MXML       1
#define XML_IMPLEMENTATION_RAPID_XML  2
#define XML_USED_IMPLEMENTATION       XML_IMPLEMENTATION_RAPID_XML

/*
 * include the wanted implementation file
 */
#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
#include <mxml.h>
#elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
#include "rapidxml-1.13/rapidxml.hpp"
#include "rapidxml-1.13/rapidxml_print.hpp"
#endif // XML_USED_IMPLEMENTATION
//
// std
#include <string>
#include <vector>

#include "vision_utils/utils/StringUtils.h"

class XmlDocument {
public:
#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
  typedef mxml_node_t Node;
#elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
  typedef rapidxml::xml_node<> Node;
#endif // XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML

  /*!
     * constructor
     * @return an empty XmlDocument
     */
  XmlDocument()  {
    maggieDebug3("ctor");
    _path = "";
    _folder = "";

#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    _root = NULL;
#elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
#endif
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     * constructor
     * @param filename the absolute path to the file to parse
     * @return the XmlDocument obtained by parsing filename
     */
  XmlDocument(const std::string & filename)
  {
    maggieDebug3("ctor");
#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    _root = NULL;
#elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
#endif
    load_from_file(filename);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~XmlDocument() {
    maggieDebug3("dtor");
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    mxmlDelete(_root);
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    doc.remove_all_nodes();
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  bool load_from_string(const std::string & file_content) {
    maggieDebug3("load_from_string('%s')", file_content.c_str());

  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    _root = mxmlLoadString(NULL, file_content.c_str(), MXML_NO_CALLBACK);
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    // make a safe-to-modify copy of input_xml
    // (you should never modify the contents of an std::string directly)
    xml_copy = std::vector<char> (file_content.begin(), file_content.end());
    xml_copy.push_back('\0');

    // parse_no_data_nodes prevents RapidXML from using the somewhat surprising
    // behavior of having both values and data nodes, and having data nodes take
    // precedence over values when printing
    // >>> note that this will skip parsing of CDATA nodes <<<
    try {
      doc.parse<rapidxml::parse_no_data_nodes>(&xml_copy[0]);
    } catch (rapidxml::parse_error & e) {
      maggiePrint("Error while parsing '%s':%s",
                  string_utils::extract(file_content).c_str(),
                  e.what());
      return false;
    }
    return true;
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  bool load_from_file(const std::string & filename) {
    maggieDebug3("load_from_file('%s')", filename.c_str());

    // store path
    _path = filename;
    _folder = filename;
    std::string::size_type slash_pos = _folder.find_last_of("/");
    if (slash_pos == std::string::npos)
      _folder = "";
    else
      _folder = _folder.substr(0, slash_pos + 1); // keep the last "/"

  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    // clean if needed
    if (root() != NULL)
      mxmlDelete(root());
  #endif

    // load file
    std::string file_content;
    bool success = StringUtils::retrieve_file(filename, file_content);
    if (!success) {
      maggiePrint("Error while retrieving file '%s'. Aborting.",
                  filename.c_str());
      return false;
    }
    return load_from_string(file_content);
  }

  //////////////////////////////////////////////////////////////////////////////

  void write_to_file(const std::string & filename) const {
    maggieDebug3("write_to_file('%s')", filename.c_str());
    StringUtils::save_file(filename, to_string());
  }

  //////////////////////////////////////////////////////////////////////////////

  Node* root() const {
#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
  return _root;
#elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
  return (Node*) &doc;
#endif
}

  // ///////////////////////////////////////////////////////////////
  //  functions for extracting from tags
  /*!
     *
     * @param start
     * @param path
     * @return
     * EX :
     \code
     * node=<foo>
     *         <bar>1</bar>
     *         <bar>2</bar>
     * </foo>
     \endcode
     * get_value()
     */
  std::string get_value(const Node* start, const std::string & path) const {
    maggieDebug3("get_value('%s')", path.c_str());
    Node* node = get_node_at_direction(start, path);
    if (node == NULL) {
      maggieDebug3("The node ''%s'' does not exist in the xml doc", path.c_str());
      return "";
    }
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    //maggieDebug2("type:%i", node->type);
    //maggieDebug2("type:'%s'", node->child->value.text.string);
    // get the child
    node = node->child;
    if (node == NULL) {
      maggieDebug2("The node ''%s'' does not have any child", path.c_str());
      return "";
    }
    // get the value
    return node->value.text.string;
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    return node->value();
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     *
     * @param start
     * @param path
     * @param default_value
     * @return
     */
  template<class _T>
  _T get_value(const Node* start, const std::string & path, _T default_value) const;


  //////////////////////////////////////////////////////////////////////////////

  // ///////////////////////////////////////////////////////////////
  // functions for extracting attributes
  /*!
     *
     * @param start where to start looking for
     * @param path the path where to search the child
     * @return the node at the given direction if it exists,
     * or NULL otherwise
     * EX :
     \code
     * node=<foo>
     *         <bar>1</bar>
     *         <bar>2</bar>
     * </foo>
     \endcode
     * get_node_at_direction(node, "foo.bar") will return the bar 1 node

     */
  Node* get_node_at_direction(const Node* start, const std::string & path) const {
    maggieDebug3("get_node_at_direction('%s')", path.c_str());
    if (start == NULL) {
      maggieDebug2("The node is NULL !");
      return NULL;
    }

    // cut the path into words
    std::vector<std::string> words;
    StringUtils::StringSplit(path, ".", &words);

    Node *current_node = (Node*) start;
    for (std::vector<std::string>::const_iterator word = words.begin(); word
         != words.end(); ++word) {
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
      current_node = mxmlFindElement(current_node, current_node,
                                     word->c_str(), NULL, NULL, MXML_DESCEND);
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
      current_node = current_node->first_node(word->c_str());
  #endif
      if (current_node == NULL) {
        maggieDebug1("We didin't find the word '%s' in the xml doc", word->c_str());
        return NULL;
      } // end if current == NULL
    } // end loop word
    return current_node;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     * @param start
     * @param path
     * @param attribute_name a wanted attribute name
     * @param attribute_value a wanted attribute value
     * @return the node at the given direction if it exists,
     * or NULL otherwise
    */
  Node* get_node_at_direction(const Node* start,
                              const std::string & path,
                              const std::string & attribute_name,
                              const std::string & attribute_value) const {
    maggieDebug3("get_node_at_direction(path:'%s', attribute_name:'%s', attribute_value:'%s')",
                 path.c_str(), attribute_name.c_str(), attribute_value.c_str());

    // get all the children
    std::vector<Node*> children_at_path;
    get_all_nodes_at_direction(start, path, children_at_path);

    // see if there is a child with the wanted attribute
    for(std::vector<Node*>::const_iterator child = children_at_path.begin();
        child < children_at_path.end() ; ++child) {
      std::string child_attribute_value = get_node_attribute(*child, attribute_name);
      //  maggieDebug2("child_attribute_value:'%s', attribute_value:'%s'",
      //               child_attribute_value.c_str(), attribute_value.c_str());
      if (child_attribute_value == attribute_value)
        return *child;
    } // end loop son

    // if we arrived here, we didn't find the child
    return NULL;
  }


  //////////////////////////////////////////////////////////////////////////////

  /*!
     *
     * @param node the node whose name we want to obtain
     * @return the name of this node
     * EX :
     * node="<foo param=3></foo>"
     * get_node_name(node) = "foo"
     */
  std::string get_node_name(const Node* node) const {
    maggieDebug2("get_node_name()");
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    if (node->type == MXML_ELEMENT)
      return node->value.element.name;
    return "";
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    return node->name();
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     * @param node the node which has attributes
     * @param attribute the name of the attribute to extract
     * @return the attribute of node if found, otherwise ""
     *
     * EX :
     * node="<foo param=3></foo>"
     * get_node_attribute(node, "param") = "3"
     * get_node_attribute(node, "non_existing") = ""
     */
  std::string get_node_attribute(const Node* node, const std::string & attribute) const {
    //maggieDebug2("get_node_attribute('%s')", attribute.c_str());
    if (node == NULL) {
      maggieDebug2("The node is NULL !");
      return "";
    }

  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    std::string ans = mxmlElementGetAttr(node, attribute.c_str());
    return ans;
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    rapidxml::xml_attribute<char>* attrib = node->first_attribute(attribute.c_str());
    if (attrib == NULL) {
      maggieDebug1("The node does not contain a '%s' attribute !", attribute.c_str());
      return "";
    }
    return attrib->value();
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     * A templated version of the string equivalent
     * @param node the node which has attributes
     * @param attribute the name of the attribute to extract
     * @param default_value what to return if we don't find
     * the given attribute
     * @return the attribute of node if found, otherwise the default value
     *
     * EX :
     * node="<foo param=3></foo>"
     * get_node_attribute(node, "param", 1) = 3
     * get_node_attribute(node, "non_existing", 1) = 1
     */
  template<class _T>
  _T get_node_attribute(const Node* node,
                        const std::string & attribute,
                        _T default_value) const {
    std::string ans_string = get_node_attribute(node, attribute);
    bool success;
    _T ans = StringUtils::cast_from_string<_T>(ans_string, success);
    return (success ? ans : default_value);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     *
     * @param start where to start looking for
     * @param path the path where are the children
     * @param ans the vector that will be populated with all the children
     * of "start" and which have a given path
     * EX :
     * \code
     * node=<foo>
     *         <bar>1</bar>
     *         <bar>2</bar>
     * </foo>
     * \endcode
     * If start=the "foo" Node and path="bar",
     * ans will contain both "bar" nodes
     */
  void get_all_nodes_at_direction(const Node* start, const std::string & path,
                                  std::vector<Node*>& ans) const {
    maggieDebug3("get_all_nodes_at_direction('%s')", path.c_str());

    // find the first sibling
    Node* sibling = get_node_at_direction(start, path);
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    Node* father = sibling->parent;
  #endif
    ans.clear();

    // find the last word
    std::vector<std::string> words;
    StringUtils::StringSplit(path, ".", &words);
    std::string last_word = words.back();

    while (sibling != NULL) {
      ans.push_back(sibling);
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
      sibling = mxmlFindElement(sibling, father, last_word.c_str(), NULL,
                                NULL, MXML_DESCEND);
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
      sibling = sibling->next_sibling(last_word.c_str());
  #endif
    } // end while sibling != NULL
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
     *
     * @param start
     * @param path
     * @param ans
     */
  template<class _T>
  void get_all_values_at_direction(const Node* start,
                                   const std::string & path,
                                   std::vector<_T>& ans) const {
    //maggieDebug2("get_all_values_at_direction('%s')", path.c_str());
    std::vector<Node*> nodes;
    get_all_nodes_at_direction(start, path, nodes);

    ans.clear();
    for (std::vector<Node*>::const_iterator node = nodes.begin(); node
         != nodes.end(); ++node) {
        //#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
        //        Node* son = (*node)->child;
        //        if (son == NULL) {
        //            printf("The node ''%s'' does not have any child\n", path.c_str());
        //            continue;
        //        }
        //        // get the value
        //        bool success;
        //        _T value =
        //                StringUtils::cast_from_string<_T>(son->value.text.string, success);
        //        if (success)
        //            ans.push_back(value);

        //#else
        //        maggieError("Not implemented");
        //#endif
        std::string value_str = get_value(*node, "");
        if (value_str != "")
          ans.push_back(StringUtils::cast_from_string<_T>(value_str));
      } // end loop node
  }


  //////////////////////////////////////////////////////////////////////////////

  /*!
     * extract all children from a given node
     * @param father where to extract the childrem
     * @param ans the vector that will be populated
     * with the children of "father"
     */
  void get_all_children(Node* father, std::vector<Node*>& ans) const {
    maggieDebug2("get_all_children()");
    ans.clear();
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    Node* current = father->child;
    while (current != NULL) {
      ans.push_back(current);
      current = current->next;
    }
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    Node* current = father->first_node();
    while (current != NULL) {
      ans.push_back(current);
      current = current->next_sibling();
    }
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
      \return a string version of the whole document
      */
  std::string to_string_node(Node* node) const {
    maggieDebug2("to_string_node()");
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    maggieError("Not implemented");
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    std::ostringstream stream;
    stream << *node;
    return stream.str();
  #endif
  }


  //////////////////////////////////////////////////////////////////////////////

  /*!
      \return a string version of the whole document
      */
  std::string to_string() const {
    maggieDebug3("to_string()");
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    maggieError("Not implemented");
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    std::ostringstream stream;
    stream << "<?xml version=\"1.0\" ?>" << std::endl;
    stream << doc;
    return stream.str();
  #endif
    //return to_string_node(root());
  }



  //////////////////////////////////////////////////////////////////////////////

  // ///////////////////////////////////////////////////////////////
  // functions for writing
  Node* add_node(Node* father, Node* node_to_add, const std::string & path) {
    maggieDebug3("add_node(path:'%s')", path.c_str());
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    maggieError("Not implemented");
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    if (path == "") {
      // append it
      father->append_node(node_to_add);
      return node_to_add;
    } else {
      // split the first word and the rest
      // example "a.b.c" -> "a" and "b.c"
      size_t dot_pos = path.find('.');
      std::string first_word = (dot_pos == std::string::npos ? path : path.substr(0, dot_pos-1));
      std::string path_end = (dot_pos == std::string::npos ? "" : path.substr(dot_pos+1));

      // add the first son if needed
      Node* first_son = get_node_at_direction(father, first_word);
      if (first_son == NULL)
        first_son = add_node(father, first_word.c_str(), "");

      // add what is left recursively
      return add_node(first_son, node_to_add, path_end);
    }
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  Node* add_node(Node* father, const std::string & node_name, const std::string & path)  {
    maggieDebug3("add_node(node_name:'%s', path:'%s')", node_name.c_str(), path.c_str());
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    maggieError("Not implemented");
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    /* make a copy of the std::string into an allocated char array */
    char* node_name_array = doc.allocate_string(node_name.c_str());
    /* create the node */
    Node* son = doc.allocate_node(rapidxml::node_element, node_name_array);

    /* call the Node version of the function */
    return add_node(father, son, path);
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  // values
  //    inline void set_node_value(Node* node, const std::string & value);
  //    template<class _T>
  //    void set_node_value(Node* node, const _T & value);

  // attributes
  void set_node_attribute(Node* node, const std::string & attr_name, const std::string & value) {
    maggieDebug3("set_node_value(attr_name:'%s', value:'%s')", attr_name.c_str(), value.c_str());
  #if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
    maggieError("Not implemented");
  #elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
    // check if we need to remove the attribute beforehand
    rapidxml::xml_attribute<char>* attrib = node->first_attribute(attr_name.c_str());
    if (attrib != NULL) {
      maggieDebug3("The node already had a '%s' attribute, removing it", attr_name.c_str());
      node->remove_attribute(attrib);
    }

    // make a copy of the std::string into an allocated char array
    char* attr_name_array = doc.allocate_string(attr_name.c_str());
    char* value_array = doc.allocate_string(value.c_str());
    // create the node
    rapidxml::xml_attribute<>* new_attr = doc.allocate_attribute(attr_name_array, value_array);
    // append it
    node->append_attribute(new_attr);
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class _T>
  void set_node_attribute(Node* node, const std::string & attr_name, const _T & value) {
    std::string value_str = StringUtils::cast_to_string<_T>(value);
    set_node_attribute(node, attr_name, value_str);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! \return the path leading to this document, or "" if undefined
      This path contains the trailing "/"
    */
  std::string get_path() const
  { return _path; }


  //////////////////////////////////////////////////////////////////////////////

  //! \return the folder containing this document, or "" if undefined
  std::string get_folder() const
  { return _folder; }


  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
private:
#if XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_MXML
  Node* _root;
#elif XML_USED_IMPLEMENTATION == XML_IMPLEMENTATION_RAPID_XML
  rapidxml::xml_document<> doc;
  std::vector<char> xml_copy;
#endif // XML_USED_IMPLEMENTATION

  //! the full path to the XML file that was parsed
  std::string _path;

  //! the folder where the XML file was parsed
  std::string _folder;
};

#endif /* XMLDOCUMENT_H_ */

