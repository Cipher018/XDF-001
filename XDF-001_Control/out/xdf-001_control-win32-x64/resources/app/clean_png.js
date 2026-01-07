const fs = require('fs');
const PNG = require('pngjs').PNG;
const path = require('path');

const src = path.resolve(__dirname, 'assets/icon.png');
const dst = path.resolve(__dirname, 'assets/icon_clean.png');

fs.createReadStream(src)
  .pipe(new PNG())
  .on('parsed', function() {
    this.pack().pipe(fs.createWriteStream(dst))
      .on('finish', () => console.log('Cleaned PNG created at:', dst));
  })
  .on('error', (err) => {
    console.error('Error cleaning PNG:', err);
    process.exit(1);
  });
