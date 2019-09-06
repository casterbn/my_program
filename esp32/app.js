const bluetooth = require('node-bluetooth');
const net = require('net');

const buffer = require('buffer');
buffer.INSPECT_MAX_BYTES = 1500;

// bluetooth config
const BluetoothAddress = '80:7D:3A:B6:58:3A';
const BluetoothChannel = 1;

// ntripcaster config
const host = 'rtk.aceinna.com';
const port = '2201';
const mountPoint = 'RTK';
const userAgent = 'NTRIP Aceinna CloudRTK 1.0';
const username = 'aceinna';
const password = 'AWN7DKT-GHC4831-M48JRTK-8HTW66S';

// create bluetooth device instance
const device = new bluetooth.DeviceINQ();

// bluetooth connection
let bleConn = null;
// ntrip connection
let ntripConn = null;

// ntrip status
let ntripIsReady = false;
// serialport status
let bleIsReady = false;

const GGADELIMITER = Buffer.from('\r\n');

device
.on('finished', () => {
  console.log('Begin to connect address: %s, channel: %d', BluetoothAddress, BluetoothChannel);
  start();
})
.on('found', (address, name) => {
  console.log('Found: ' + address + ' with name ' + name);
})

device.scan();

// app start
function start() {
  connectBluetooth();
  connectNtripcaster();
}

// make a do once function
function doOnce(func) {
  let done = false;
  return function innerOnce() {
    if (done) {
      return;
    }
    done = true;
    func.call(this);
  };
}

// sleep function
function sleep(ms) {
  return new Promise((resolve, reject) => {
    setTimeout(() => {
      resolve();
    }, ms);
  });
}

// init bluetooth
function connectBluetooth() {
  if (bleConn) {
    bleConn.removeAllListeners();
    bleConn = null;
  }

  let bleBuf = Buffer.alloc(0);

  const doneOnce = doOnce(async () => {
    await sleep(1000);
    connectBluetooth();
  });

  bluetooth.connect(BluetoothAddress, BluetoothChannel, (err, connection) => {
    if (err) {
      console.error(err);
      doneOnce();
      return;
    }
    console.log('Bluetooth connect success');

    bleConn = connection;
    bleIsReady = true;

    connection.on('data', (data) => {
      bleBuf = Buffer.concat([bleBuf, data]);
      while(true) {
        const idx = bleBuf.indexOf(GGADELIMITER);
        if (idx < 0) {
          return;
        }
        const gga = bleBuf.slice(0, idx + GGADELIMITER.length);
        writeToNtrip(gga);
        bleBuf = bleBuf.slice(idx + GGADELIMITER.length);
      }
    });

    connection.on('close', () => {
      bleIsReady = false;
      console.log('Bluetooth is close');
      doneOnce();
    });
    
    connection.on('error', err => {
      bleIsReady = false;
      console.error('Bluetooth error', err);
      doneOnce();
    });
  });
}

// init ntripcaster connect
function connectNtripcaster() {
  if (ntripConn) {
    ntripConn.removeAllListeners();
    ntripConn = null;
  }

  ntripConn = net.createConnection({
    host,
    port
  }, () => {
    const authorization = Buffer.from(
      username + ':' + password,
      'utf8'
    ).toString('base64');
    const data = `GET /${mountPoint} HTTP/1.0\r\nUser-Agent: ${userAgent}\r\nAuthorization: Basic ${authorization}\r\n\r\n`;
    ntripConn.write(data);
  });
  
  ntripConn.on('data', data => {
    if (ntripIsReady) {
      writeToPort(data);
      return;
    }
  
    if (data.toString() === 'ICY 200 OK\r\n\r\n') {
      ntripIsReady = true;
    }
  });

  const doneOnce = doOnce(async () => {
    await sleep(1000);
    connectNtripcaster();
  });
  
  ntripConn.on('close', () => {
    ntripIsReady = false;
    console.log('ntripclient is close');
    doneOnce();
  });
  
  ntripConn.on('error', err => {
    ntripIsReady = false;
    console.error('ntripclient error', err);
    doneOnce();
  });  
}

// write data to ntripcaster
function writeToNtrip(data) {
  if (!ntripIsReady && ntripConn) {
    console.log('ntripclient is not ready');
    return;
  }
  console.log(data.toString());
  ntripConn.write(data);
}

// write data to bluetooth
function writeToPort(data) {
  if (!bleIsReady && bleConn) {
    console.log('Bluetooth is not ready');
    return;
  }
  console.log(new Date().toUTCString(), data);
  bleConn.write(data);
}
