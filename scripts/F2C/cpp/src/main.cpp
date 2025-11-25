/**
 * @file main.cpp
 * @brief Entry point for F2C Coverage Planner GUI
 * 
 * C++ version of the Fields2Cover coverage planner GUI.
 * Equivalent to f2c_gui.py but with native performance.
 */

#include <QApplication>
#include <QStyleFactory>
#include <QFont>
#include <QFontDatabase>
#include <iostream>

#include "coverage_gui.hpp"

int main(int argc, char* argv[])
{
    // Create Qt application
    QApplication app(argc, argv);
    
    // Set application metadata
    app.setApplicationName("F2C Coverage Planner");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("PilotControl");
    
    // Use Fusion style for modern look
    app.setStyle(QStyleFactory::create("Fusion"));
    
    // Set default font
    QFont font("Segoe UI", 10);
    QFontDatabase fontDb;
    if (fontDb.families().filter("Segoe UI").isEmpty()) {
        font = QFont("Sans Serif", 10);
    }
    app.setFont(font);
    
    // Print startup info
    std::cout << "=== F2C Coverage Planner (C++) ===" << std::endl;
    std::cout << "Version: 1.0.0" << std::endl;
    std::cout << "Qt Version: " << QT_VERSION_STR << std::endl;
#ifdef HAVE_FIELDS2COVER
    std::cout << "Fields2Cover: Available" << std::endl;
#else
    std::cout << "Fields2Cover: NOT FOUND (coverage features disabled)" << std::endl;
#endif
#ifdef HAVE_CGAL
    std::cout << "CGAL: Available (alphashape enabled)" << std::endl;
#else
    std::cout << "CGAL: NOT FOUND (using convex hull fallback)" << std::endl;
#endif
    std::cout << "==================================" << std::endl;
    
    // Create and show main window
    f2c_cpp::CoverageGUI gui;
    gui.show();
    
    // Run event loop
    return app.exec();
}

