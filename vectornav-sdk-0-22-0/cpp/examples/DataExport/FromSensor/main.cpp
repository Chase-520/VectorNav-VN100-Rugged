// The MIT License (MIT)
// 
// VectorNav SDK (v0.22.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <filesystem>
#include <thread>

#include "vectornav/Config.hpp"
#include "vectornav/ExporterCsv.hpp"
// #include "FileExportFaDispatcher.hpp"
#include "vectornav/Implementation/AsciiPacketDispatcher.hpp"
#include "vectornav/Implementation/FaPacketDispatcher.hpp"
#include "vectornav/Implementation/FaPacketProtocol.hpp"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/Interface/Sensor.hpp"

using namespace VN;
namespace fs = std::filesystem;

Sensor sensor;

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
{
    const std::string portName = (argc > 1) ? argv[1] : "COM18";
    const std::string outputDirectory = (argc > 2) ? argv[2] : (fs::path(__FILE__).parent_path()).string();

    auto latestError = sensor.autoConnect(portName);
    if (latestError != Error::None)
    {
        std::cout << "Error " << latestError << " encountered when connecting to " << sensor.connectedPortName().value() << ".\t" << std::endl;
        return static_cast<int>(latestError);
    }
    std::cout << "Connected to " << portName << " at " << sensor.connectedBaudRate().value() << std::endl;

    ExporterCsv csvExporter(outputDirectory);

    // Add a subscriber to all VN FA and ASCII packets
    latestError = sensor.subscribeToMessage(csvExporter.getQueuePtr(), Sensor::BinaryOutputMeasurements{}, Sensor::FaSubscriberFilterType::AnyMatch);
    if (latestError != Error::None)
    {
        std::cout << "Error " << latestError << " encountered when subscribing." << std::endl;
        return static_cast<int>(latestError);
    }
    latestError = sensor.subscribeToMessage(csvExporter.getQueuePtr(), "VN", Sensor::AsciiSubscriberFilterType::StartsWith);
    if (latestError != Error::None)
    {
        std::cout << "Error " << latestError << " encountered when subscribing." << std::endl;
        return static_cast<int>(latestError);
    }

    csvExporter.start();
    std::cout << "Logging to " << outputDirectory << std::endl;

    std::this_thread::sleep_for(5s);
    csvExporter.stop();
    std::cout << "ExportFromSensor example complete." << std::endl;
}
