/***************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the QtBluetooth module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "devicehandler.h"
#include "deviceinfo.h"
#include <QtEndian>
#include <QRandomGenerator>

#include <QDebug> //just for debugging

#include <InsoleSensorData.hpp>

// Copied from service uuid in smoles-firmware
quint128 service_uuid({0xab,0x08,0x28,0xb1,0x19,0x8e,0x43,0x51,0xb7,0x79,0x90,0x1f,0xa0,0xe0,0x37,0x1e});
quint128 characteristic_uuid({0x09,0x72,0xEF,0x8C,0x76,0x13,0x40,0x75,0xAD,0x52,0x75,0x6F,0x33,0xD4,0xDA,0x91});

DeviceHandler::DeviceHandler(QObject *parent) :
    BluetoothBaseClass(parent),
    m_foundSmoleService(false),
    m_measuring(false)
{
}

void DeviceHandler::setAddressType(AddressType type)
{
    switch (type) {
    case DeviceHandler::AddressType::PublicAddress:
        m_addressType = QLowEnergyController::PublicAddress;
        break;
    case DeviceHandler::AddressType::RandomAddress:
        m_addressType = QLowEnergyController::RandomAddress;
        break;
    }
}

DeviceHandler::AddressType DeviceHandler::addressType() const
{
    if (m_addressType == QLowEnergyController::RandomAddress)
        return DeviceHandler::AddressType::RandomAddress;

    return DeviceHandler::AddressType::PublicAddress;
}

void DeviceHandler::setDevice(DeviceInfo *device)
{
    clearMessages();
    m_currentDevice = device;

    // Disconnect and delete old connection
    if (m_control) {
        m_control->disconnectFromDevice();
        delete m_control;
        m_control = nullptr;
    }

    // Create new controller and connect it if device available
    if (m_currentDevice) {

        // Make connections
        //! [Connect-Signals-1]
        m_control = QLowEnergyController::createCentral(m_currentDevice->getDevice(), this);
        //! [Connect-Signals-1]
        m_control->setRemoteAddressType(m_addressType);
        //! [Connect-Signals-2]
        connect(m_control, &QLowEnergyController::serviceDiscovered,
                this, &DeviceHandler::serviceDiscovered);
        connect(m_control, &QLowEnergyController::discoveryFinished,
                this, &DeviceHandler::serviceScanDone, Qt::QueuedConnection);

        connect(m_control, static_cast<void (QLowEnergyController::*)(QLowEnergyController::Error)>(&QLowEnergyController::error),
                this, [this](QLowEnergyController::Error error) {
            Q_UNUSED(error);
            setError("Cannot connect to remote device.");
        });
        connect(m_control, &QLowEnergyController::connected, this, [this]() {
            setInfo("Controller connected. Search services...");
            qInfo("Search for services1");
            m_control->discoverServices();
            auto message = QStringLiteral("Total services: %1").arg(m_control->services().size());
            qInfo(qUtf8Printable(message));
            for (auto service : m_control->services())
            {
                qInfo(qUtf8Printable(service.toString()));
            }
        });
        connect(m_control, &QLowEnergyController::disconnected, this, [this]() {
            setError("LowEnergy controller disconnected");
        });

        // Connect
        m_control->connectToDevice();        
        //! [Connect-Signals-2]
    }
}

void DeviceHandler::startMeasurement()
{
    if (alive()) {
        m_start = QDateTime::currentDateTime();
        m_measuring = true;
        first_measurement_expected = true;
        file.open("test.txt");
        file << "{\n" << "\"SensorDataPackage\":[\n";
        emit measuringChanged();
    }
}

void DeviceHandler::stopMeasurement()
{
    m_measuring = false;
    // todo: remove last character because it is a comma
    file << "]\n" << "}";
    file.close();
    emit measuringChanged();
}

//! [Filter HeartRate service 1]
void DeviceHandler::serviceDiscovered(const QBluetoothUuid &gatt)
{
    if (gatt == QBluetoothUuid(service_uuid))
    {
        m_foundSmoleService = true;
        qInfo("gatt matched");
    }
    else
    {
        qInfo("gatt did not match");
    }
    
}
//! [Filter HeartRate service 1]

void DeviceHandler::serviceScanDone()
{
    setInfo("Service scan done.");

    // Delete old service if available
    if (m_service) {
        delete m_service;
        m_service = nullptr;
    }

    qInfo("Check if smoles service is available");
//! [Filter HeartRate service 2]
    // If heartRateService found, create new service
    if (m_foundSmoleService)
    {
        qInfo("Somle service found");
        m_service = m_control->createServiceObject(QBluetoothUuid(service_uuid), this);
    }
    else
    {
        qWarning("Smole service not found.");
    }

    if (m_service) {
        connect(m_service, &QLowEnergyService::stateChanged, this, &DeviceHandler::serviceStateChanged);
        connect(m_service, &QLowEnergyService::characteristicChanged, this, &DeviceHandler::updatePressureValue);
        connect(m_service, &QLowEnergyService::descriptorWritten, this, &DeviceHandler::confirmedDescriptorWrite);
        m_service->discoverDetails();
    }
//! [Filter HeartRate service 2]
}

// Service functions
//! [Find HRM characteristic]
void DeviceHandler::serviceStateChanged(QLowEnergyService::ServiceState s)
{
    qInfo("service state changed");
    switch (s) {
    case QLowEnergyService::DiscoveringServices:
        setInfo(tr("Discovering services..."));
        break;
    case QLowEnergyService::ServiceDiscovered:
    {
        setInfo(tr("Service discovered."));

        const QLowEnergyCharacteristic hrChar = m_service->characteristic(QBluetoothUuid(characteristic_uuid));
        if (!hrChar.isValid()) {
            break;
        }

        m_notificationDesc = hrChar.descriptor(QBluetoothUuid::ClientCharacteristicConfiguration);
        if (m_notificationDesc.isValid())
            m_service->writeDescriptor(m_notificationDesc, QByteArray::fromHex("0100"));

        break;
    }
    default:
        //nothing for now
        break;
    }

    emit aliveChanged();
}
//! [Find HRM characteristic]

//! [Reading value]
void DeviceHandler::updatePressureValue(const QLowEnergyCharacteristic &c, const QByteArray &value)
{
    // ignore any other characteristic change -> shouldn't really happen though
    if (c.uuid() != QBluetoothUuid(characteristic_uuid))
        return;

    QString recieved_data = value.constData();
    std::string recieved_string = recieved_data.toStdString();
    addMeasurement(recieved_string);

    setError("Start json parse");
    smoles::InsoleSensorData sensor_data = nlohmann::json::parse(recieved_string);
    setError("Json parse finished");

    std::string output = "Timestamp " + std::to_string(sensor_data.get_time_stamp())
      + ", Left foot " + std::to_string(sensor_data.get_left_foot());
    setInfo(QString::fromStdString(output));
}
//! [Reading value]

void DeviceHandler::confirmedDescriptorWrite(const QLowEnergyDescriptor &d, const QByteArray &value)
{
    if (d.isValid() && d == m_notificationDesc && value == QByteArray::fromHex("0000")) {
        //disabled notifications -> assume disconnect intent
        m_control->disconnectFromDevice();
        delete m_service;
        m_service = nullptr;
    }
}

void DeviceHandler::disconnectService()
{
    m_foundSmoleService = false;

    //disable notifications
    if (m_notificationDesc.isValid() && m_service
            && m_notificationDesc.value() == QByteArray::fromHex("0100")) {
        m_service->writeDescriptor(m_notificationDesc, QByteArray::fromHex("0000"));
    } else {
        if (m_control)
            m_control->disconnectFromDevice();

        delete m_service;
        m_service = nullptr;
    }
}

bool DeviceHandler::measuring() const
{
    return m_measuring;
}

bool DeviceHandler::alive() const
{
    if (m_service)
        return m_service->state() == QLowEnergyService::ServiceDiscovered;

    return false;
}

void DeviceHandler::addMeasurement(const std::string &_measurement)
{
    if (!first_measurement_expected)
    {
        file << ",\n" + _measurement;
    }
    else
    {
        file << _measurement;
        first_measurement_expected = false;
    }
    emit statsChanged();
}
