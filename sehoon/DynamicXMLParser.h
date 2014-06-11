#ifndef SEHOON_DYNAMIC_XML_PARSER_H
#define SEHOON_DYNAMIC_XML_PARSER_H

#include <iostream>
#include <cstdlib>
#include <string>

#include "ticpp.h"
#include <glog/logging.h>
#include <Eigen/Dense>
#include "Moreeigen.h"
#include <boost/lexical_cast.hpp>

#include "ExprParser.h"

namespace sehoon {
    namespace xml {
        /*
          DynamicXMLParser* xml = new DynamicXMLParser("test.xml");

          // Case 1. <controller name="airborneFB" />
          string controllerName = xml->readString("controller@name");
          // Case 2. <skeleton file="./Data/skel/Nick01a.skel">
          //             <pose> 0 0 0.0 1.0 ....   </pose> 
          //             <pose> 0 0 0.3 0.9 ....   </pose> 
          int numPoses = xml->countNode("skeleton.pose");
          Eigen::VectorXd pose0 = xml->readVectorXd("skeleton.pose", 0);
          Eigen::VectorXd pose1 = xml->readVectorXd("skeleton.pose", 1);
          
         */

        class DynamicXMLParser {
        public:
            DynamicXMLParser()
                : doc(NULL) {
            }
            
            DynamicXMLParser(const char* const filename)
                : doc(NULL) {
                load(filename);
            }

            ~DynamicXMLParser() {
                destroy();
            }

            bool load(const char* const filename) {
                using google::INFO;
                using google::ERROR;
                try {
                    doc = new ticpp::Document(filename);
                    doc->LoadFile();
                    LOG(ERROR) << "Load xml [" << filename << "] OK";
                } catch(ticpp::Exception e) {
                    doc = NULL;
                    LOG(ERROR) << "Fail to load xml [" << filename << "]";
                    LOG(ERROR) << "Exception: " << e.what();
                    return false;
                }
                return true;
            }

            void destroy() {
                delete doc;
            }

            int count(const std::string& name) const {
                std::string path(name);
                ticpp::Element* element = getElement(path, 0);

                int count = 0;
                while(element != NULL) {
                    count++;
                    element = element->NextSiblingElement(element->Value(), false);
                }
                return count;
            }

            bool isExists(const char* const name, int index = 0) const {
                using google::INFO;
                std::string path(name);
                ticpp::Element* element = getElement(path, index);

                for (int i = 0; i < index; i++) {
                    if (element == NULL) {
                        return false;
                    }
                    element = element->NextSiblingElement(element->Value(), false);
                }
                if (element == NULL) {
                    return false;
                }


                if (path != "") {
                    if (!hasAttribute(element, path)) {
                        return false;
                    }
                    return true;
                } else {
                    return true;
                }
            }


            std::string readString(const char* const name, int index = 0) const {
                using google::INFO;
                std::string path(name);
                ticpp::Element* element = getElement(path, index);

                for (int i = 0; i < index; i++) {
                    if (element == NULL) {
                        return std::string("");
                    }
                    element = element->NextSiblingElement(element->Value(), false);
                }

                if (path != "") {
                    return getAttribute(element, path);
                } else {
                    return getText(element);
                }
            }
            
            bool readBool(const char* const name, int index = 0) const {
                std::string ret = readString(name, index);
                return toBool(ret);
            }

            int readInt(const char* const name, int index = 0) const {
                std::string ret = readString(name, index);
                return toInt(ret);
            }

            double readDouble(const char* const name, int index = 0) const {
                std::string ret = readString(name, index);
                return toDouble(ret);
            }

            double readExpr(const char* const name, int index = 0) {
                std::string ret = readString(name, index);
                parser.parse(ret.c_str());
                return parser.getAnswer();
            }

            Eigen::VectorXd readVectorXd(const char* const name, int index = 0) const {
                std::string ret = readString(name, index);
                return toVectorXd(ret);
            }

            Eigen::VectorXd readVector3d(const char* const name, int index = 0) const {
                std::string ret = readString(name, index);
                return toVector3d(ret);
            }

            
            ticpp::Document* getDocument() const {
                return doc;
            }

            sehoon::expr::Variablelist* getVariableList() {
                return parser.getVarList();
            }

            ticpp::Element*  getElement(std::string& path, int index = 0) const {
                using google::INFO;
                using google::ERROR;
                path = "." + path;
                ticpp::Node* node = dynamic_cast<ticpp::Node*>(getDocument());

                while(path.length() > 0) {
                    std::string cmd   = removeFirstCharacter(path);
                    std::string token = removeFirstToken(path);

                    // LOG(INFO) << "cmd = [" << cmd << "]";
                    // LOG(INFO) << "token = [" << token << "]";

                    if (cmd == "@") {
                        path = token;
                        // LOG(INFO) << "result path = " << path;
                        return dynamic_cast<ticpp::Element*>(node);
                    } else if (cmd == ".") {
                        try {
                            // node = getDocument()->FirstChildElement("controller");
                            node =  dynamic_cast<ticpp::Node*>(
                                node->FirstChildElement(token) );
                        } catch(ticpp::Exception e) {
                            LOG(INFO) << "cmd = " << cmd << " "
                                      << "token = " << token << " "
                                      << "path = " << path;
                            LOG(ERROR) << "ticpp exception: " << e.what();
                            return NULL;
                        }
                    } else {
                        LOG(ERROR) << "xml parse error: " << cmd << ". token = " << token;
                    }


                }
                path = "";
                return dynamic_cast<ticpp::Element*>(node);
            }

            std::string getText(ticpp::Element* element) const {
                return element->GetText();
            }

            std::string getAttribute(ticpp::Element* element, const std::string& name) const {
                std::string attr;
                element->GetAttribute(name, &attr);
                return attr;
            }

            bool hasAttribute(ticpp::Element* element, const std::string& name) const {
                return element->HasAttribute(name);
            }


            static int toInt(const std::string& str) {
                return boost::lexical_cast<int>(str);
            }

            static double toDouble(const std::string& str) {
                double ret = 0.0;
                try {
                    ret = boost::lexical_cast<double>(str);
                } catch(boost::bad_lexical_cast &) {
                    using google::ERROR;
                    LOG(ERROR) << "Bad lexical_cast to double : " << str;
                }
                return ret;
            }

            static Eigen::VectorXd toVectorXd(const std::string& str) {
                return sehoon::moreeigen::convertStringToVectorXd(str.c_str());
            }

            static Eigen::VectorXd toVector3d(const std::string& str) {
                Eigen::VectorXd v = toVectorXd(str);
                CHECK_EQ(v.size(), 3);
                return Eigen::Vector3d(v(0), v(1), v(2));
            }

            static bool toBool(const std::string& str) {
                using google::ERROR;

                if (strcasecmp(str.c_str(), "1") == 0 ||
                    strcasecmp(str.c_str(), "true") == 0 ||
                    strcasecmp(str.c_str(), "t") == 0) {
                    return true;
                }
                if (strcasecmp(str.c_str(), "0") == 0 ||
                    strcasecmp(str.c_str(), "false") == 0 ||
                    strcasecmp(str.c_str(), "f") == 0) {
                    return false;
                }
                LOG(ERROR) << "Invalid string to bool cast: (true is assumed)" << str;
                return true;
            }
        private:
            static std::string removeFirstCharacter(std::string& str) {
                int index = 1;
                std::string ch = str.substr(0, index);
                str = str.substr(index);
                return ch;
            }

            static std::string removeFirstToken(std::string& str) {
                int index = str.length();
                for (int i = 0; i < str.length(); i++) {
                    if (str[i] == '.' || str[i] == '@') {
                        index = i;
                        break;
                    }
                }
                
                std::string token = str.substr(0, index);
                str = str.substr(index);
                return token;
            }

            ticpp::Document* doc;
            sehoon::expr::Parser parser;
        };
        
    } // namespace xml
} // namespace sehoon

#endif // #ifndef DYNAMIC_XML_PARSER_H

