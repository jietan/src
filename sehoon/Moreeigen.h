#ifndef SEHOON_MOREEIGEN_H
#define SEHOON_MOREEIGEN_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>

namespace sehoon {
    namespace moreeigen {
        static Eigen::VectorXd toEigen(const std::vector<double>& v);
        static std::vector<double> toStl(const Eigen::VectorXd& v);
        static Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& m);
        static Eigen::MatrixXd pseudoinverse2(const Eigen::MatrixXd& m);
        class IO;
        static Eigen::VectorXd convertStringToVectorXd(const char* const str);
        static std::string convertVectorXdToString(const Eigen::VectorXd& v);

        static Eigen::VectorXd toEigen(const std::vector<double>& v) {
            Eigen::VectorXd ret( v.size() );
            for (int i = 0; i < v.size(); i++) {
                ret(i) = v[i];
            }
            return ret;
        }

        static std::vector<double> toStl(const Eigen::VectorXd& v) {
            std::vector<double> ret;
            ret.resize(v.size());
            for (int i = 0; i < v.size(); i++) {
                ret[i] = v(i);
            }
            return ret;
        }

        //////////////////////////////////////////////////
        //
        static Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& m) {
            Eigen::JacobiSVD<Eigen::MatrixXd>
                svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXd U = svd.matrixU();
            Eigen::MatrixXd V = svd.matrixV();
            Eigen::VectorXd diag = svd.singularValues();
            Eigen::MatrixXd D = Eigen::MatrixXd::Zero(diag.size(), diag.size());
            D.diagonal() = diag;
            // LOG(INFO) << "mul = " << endl << U * D * V.transpose();
            // LOG(INFO) << "U = " << endl << U;
            // LOG(INFO) << "D = " << endl << D;
            // LOG(INFO) << "V = " << endl << V;
            // exit(0);
			Eigen::MatrixXd UTrans = U.transpose();
			Eigen::MatrixXd Dinv = D.inverse();
            return V * Dinv * UTrans;
        }
        // static Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& m)
        //////////////////////////////////////////////////

        //////////////////////////////////////////////////
        //
        static Eigen::MatrixXd pseudoinverse2(const Eigen::MatrixXd& m) {
            const Eigen::MatrixXd mT = m.transpose();
            const Eigen::MatrixXd mTm = mT * m;
            // LOG(INFO) << "mTm = " << endl << mTm;
            const Eigen::MatrixXd mTmInv = mTm.inverse();
            const Eigen::MatrixXd mTmInvmT = mTmInv * mT;
            return mTmInvmT;
        }
        // static Eigen::MatrixXd pseudoinverse2(const Eigen::MatrixXd& m)
        //////////////////////////////////////////////////

        //////////////////////////////////////////////////
        //
        static bool ioPrintLeadingSize = false;

        class IO {
        public:
            IO(const Eigen::Vector3d& _v) : v(&_v), size(3) {
            }
            IO(Eigen::Vector3d& _v) : v(&_v), size(3) {
            }
            IO(const Eigen::VectorXd& _v) : v(&_v), size(-1) {

            }
            IO(Eigen::VectorXd& _v) : v(&_v), size(-1) {
            }

            friend std::ostream& operator << (std::ostream& out, const IO& io) {
                out << "[";
                if (io.size == -1) {
                    const Eigen::VectorXd* const v(
                        static_cast<const Eigen::VectorXd* const>(io.v)
                        );

                    if (true) {
                        out << v->size();
                        for (int i = 0; i < v->size(); i++) {
                            out << " " << (*v)(i);
                        }
                    } else {
                        for (int i = 0; i < v->size(); i++) {
                            if (i != 0) out << " ";
                            out << (*v)(i);
                        }
                    }
                } else if (io.size == 3) {
                    const Eigen::Vector3d* const v(
                        static_cast<const Eigen::Vector3d* const>(io.v)
                        );
                    out << (*v)(0);
                    for (int i = 1; i < v->size(); i++) {
                        out << " " << (*v)(i);
                    }

                }

                out << "]";
                return out;
            }

            friend std::istream& operator >> (std::istream& in, const IO& io) {
                char temp;
                in >> temp;
                if (io.size == -1) {
                    Eigen::VectorXd* v(
                        const_cast<Eigen::VectorXd*>(
                            static_cast<const Eigen::VectorXd* const>(io.v)
                            )
                        );
                    int size;
                    in >> size;
                    v->resize(size);
                    for (int i = 0; i < v->size(); i++) {
                        in >> (*v)(i);
                    }
                } else if (io.size == 3) {
                    Eigen::Vector3d* v(
                        const_cast<Eigen::Vector3d*>(
                            static_cast<const Eigen::Vector3d* const>(io.v)
                            )
                        );
                    for (int i = 0; i < v->size(); i++) {
                        in >> (*v)(i);
                    }

                }
                in >> temp;
                return in;
            }
            
        private:
            const void* v;
            int size;
        };

        class IOPrintSizeType {
            friend std::ostream& operator << (std::ostream& out, const IOPrintSizeType& p) {
                ioPrintLeadingSize = true;
                return out;
            }
        };

        class IONoPrintSizeType {
            friend std::ostream& operator << (std::ostream& out, const IONoPrintSizeType& p) {
                ioPrintLeadingSize = false;
                return out;
            }
        };

        static IOPrintSizeType IOPrintSize;
        static IONoPrintSizeType IONoPrintSize;


        // class IO
        //////////////////////////////////////////////////

        //////////////////////////////////////////////////
        //
        Eigen::VectorXd convertStringToVectorXd(const char* const str) {
            std::stringstream stream(str);
            std::vector<double> temp;
            while(true) {
                double value;
                stream >> value;
                if (stream.fail()) {
                    break;
                }
                temp.push_back(value);
            }

            Eigen::VectorXd ret(temp.size());
            for (int i = 0; i < ret.size(); i++) {
                ret(i) = temp[i];
            }
            return ret;
        }
        // Eigen::VectorXd convertStringToVectorXd(const char* const str)
        //////////////////////////////////////////////////

        static std::string convertVectorXdToString(const Eigen::VectorXd& v) {
            std::stringstream stream("");
            stream << "[";
            for (int i = 0; i < v.size(); i++) {
                if (i != 0) stream << ' ';
                stream << v(i);
            }
            stream << "]";
            return stream.str();
        }



    } // namespace moreeigen
} // namespace sehoon

#endif // #ifndef SEHOON_MOREEIGEN_H

