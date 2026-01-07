const fs = require('fs');
const path = require('path');
const jpeg = require('jpeg-js');
const PNG = require('pngjs').PNG;
const pngToIco = require('png-to-ico').default;

const srcPath = path.resolve(__dirname, 'assets/icon.png');
const pngPath = path.resolve(__dirname, 'assets/icon_fixed.png');
const icoPath = path.resolve(__dirname, 'assets/icon.ico');

async function run() {
  try {
    const jpegData = fs.readFileSync(srcPath);
    const rawImageData = jpeg.decode(jpegData);
    
    const png = new PNG({
      width: rawImageData.width,
      height: rawImageData.height,
    });
    png.data = rawImageData.data;
    
    const pngBuffer = PNG.sync.write(png);
    fs.writeFileSync(pngPath, pngBuffer);
    console.log('Successfully created intermediate PNG:', pngPath);
    
    const icoBuffer = await pngToIco(pngPath);
    fs.writeFileSync(icoPath, icoBuffer);
    console.log('Successfully created final ICO:', icoPath);
  } catch (err) {
    console.error('Failure:', err);
    process.exit(1);
  }
}

run();
