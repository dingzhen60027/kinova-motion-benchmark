/* Inspect saved local replays after all measured planning and audits finish.
 * Uses the host's existing Playwright bundle; no environment installation.
 */
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const {pathToFileURL} = require('node:url');
const {chromium} = require('playwright');

const base = __dirname;
const read = p => JSON.parse(fs.readFileSync(p, 'utf8'));
const hash = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const check = (v, message) => { if (!v) throw new Error(message); };
const write = (p, data) => fs.writeFileSync(p, JSON.stringify(data, null, 2)+'\n', {flag:'wx'});

async function main() {
  check(process.argv.includes('--planning-finished'), 'Wait until measured planning finishes');
  const completed = read(path.join(base, 'completed_blocks.json'));
  check(completed.complete === true && completed.completed_jobs === 380, 'Incomplete study');
  for (const name of ['comparison']) {
    const audit = read(path.join(base, name+'_audit.json'));
    check(audit.passed === true && audit.errors.length === 0, 'Unaccepted '+name+' audit');
  }
  const replayRoot = path.join(base, 'replays');
  const indexFile = path.join(replayRoot, 'index.json');
  const index = read(indexFile);
  check(index.complete === true && index.failures.length === 0, 'Unaccepted replay generation');
  const output = path.join(base, 'browser_checks_v1');
  fs.mkdirSync(output); // Refuse to replace previous verification evidence.
  const record = {passed:false, created_utc:new Date().toISOString(),
    command:process.argv, node_path:process.env.NODE_PATH, script_sha256:hash(__filename),
    replay_index_sha256:hash(indexFile), executable:chromium.executablePath(),
    executable_sha256:hash(chromium.executablePath()), views:[], errors:[]};
  let browser;
  try {
    browser = await chromium.launch({headless:true, args:['--disable-gpu']});
    record.browser_version = browser.version();
    for (const item of index.illustrations) {
      if (item.status !== 'created' || item.reuse_of) continue;
      const folder = path.join(replayRoot, item.name);
      const source = path.join(folder, 'replay.html');
      const before = hash(source);
      const view = {name:item.name, source, source_sha256:before, passed:false, page_errors:[]};
      record.views.push(view);
      const page = await browser.newPage({viewport:{width:1280,height:1000}, deviceScaleFactor:1});
      page.on('pageerror', e => view.page_errors.push(String(e)));
      page.on('requestfailed', r => view.page_errors.push(r.url()+': '+r.failure()?.errorText));
      try {
        await page.goto(pathToFileURL(source).href, {waitUntil:'load'});
        await page.waitForFunction(() => document.querySelector('#stamp')?.textContent.includes(' / '));
        const data = await page.evaluate(() => {
          const d=JSON.parse(document.querySelector('#data').textContent);
          return {case_id:d.case_id, status:d.status, role:d.display_role,
            frames:d.frames.length, sample_source:d.sample_source,
            has_trajectory:d.has_trajectory, first:d.frames[0].q, last:d.frames.at(-1).q,
            missing_images:[...document.images].filter(i=>!i.complete || !i.naturalWidth).length};
        });
        view.data = data;
        check(data.case_id===item.case_id && data.status===item.source_status, 'Source identity mismatch');
        check(data.role===item.role && data.has_trajectory && data.frames>1, 'Wrong trajectory provenance');
        check(data.missing_images===0, 'Missing plot image');
        const slider=page.locator('#slider');
        await slider.focus(); await slider.press('End');
        check(Number(await slider.inputValue())===data.frames-1, 'Slider End failed');
        check((await page.locator('#state').textContent()).includes(data.last.map(v=>v.toFixed(5)).join(', ')),
          'Displayed final joint state differs from saved samples');
        await slider.press('Home');
        check(Number(await slider.inputValue())===0, 'Slider Home failed');
        await page.locator('#play').click();
        await page.waitForFunction(() => +document.querySelector('#slider').value>0);
        await page.locator('#play').click();
        check((await page.locator('#play').textContent())==='Play', 'Playback pause failed');
        await page.locator('#focus').click();
        check((await page.locator('#focus').textContent())==='Focus robot', 'Full-scene view failed');
        await page.locator('#focus').click();
        check((await page.locator('#focus').textContent())==='Full scene', 'Robot view failed');
        await page.locator('#yaw').focus(); await page.locator('#yaw').press('ArrowRight');
        check(Number(await page.locator('#yaw').inputValue())===-51, 'View rotation failed');
        view.canvas = await page.evaluate(() => {
          const c=document.querySelector('#view'), a=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
          let opaque=0,blue=0;
          for(let i=0;i<a.length;i+=4) if(a[i+3]>100) {
            opaque++; if(a[i]<130 && a[i+2]>100 && a[i+2]>a[i+1]*1.15) blue++;
          }
          return {width:c.width,height:c.height,opaque_pixels:opaque,blue_pixels:blue};
        });
        check(view.canvas.opaque_pixels>200 && view.canvas.blue_pixels>30, 'Canvas did not draw the model');
        check(view.page_errors.length===0, 'Browser errors');
        const screenshot=path.join(output,item.name+'.png');
        await page.screenshot({path:screenshot,fullPage:true});
        view.screenshot=screenshot;view.screenshot_sha256=hash(screenshot);
        check(hash(source)===before, 'Replay source modified during inspection');
        view.passed=true;
      } catch (e) { view.error=String(e.stack||e);record.errors.push(item.name+': '+String(e)); }
      finally { await page.close(); }
    }
    check(record.views.length>=5, 'Missing method replays');
    check(hash(indexFile)===record.replay_index_sha256, 'Replay index changed');
    record.passed=record.errors.length===0 && record.views.every(v=>v.passed);
  } catch (e) { record.errors.push(String(e.stack||e)); }
  finally { if (browser) await browser.close(); }
  write(path.join(output,'inspection.json'),record);
  console.log(JSON.stringify({passed:record.passed,views:record.views.length,errors:record.errors},null,2));
  if (!record.passed) process.exitCode=1;
}

main().catch(e=>{console.error(e.stack||e);process.exitCode=1;});
