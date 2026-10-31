/* Actual post-planning browser interactions against loopback-only saved HTML. */
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const { chromium } = require('/home/eric/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');

const origin = process.argv[2];
const out = path.resolve(process.argv[3]);
const gallery = path.resolve(__dirname, 'formal_replays_v1');
if (!/^http:\/\/127\.0\.0\.1:\d+$/.test(origin)) throw Error('Loopback HTTP origin required');
if (fs.existsSync(out)) throw Error('Use a fresh evidence directory; do not overwrite');
if (!out.startsWith(__dirname + path.sep)) throw Error('Evidence must remain under reports/baseline_study');
fs.mkdirSync(out, { recursive: true });
const hash = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const fileHash = file => hash(fs.readFileSync(file));
const read = file => JSON.parse(fs.readFileSync(file, 'utf8'));
const report = {
  passed: false, command: [process.execPath, ...process.argv.slice(1)], origin,
  started_utc: new Date().toISOString(),
  cua_probe: { available: false, iab_error: 'Browser is not available: iab', inventory: { apps: [], browsers: [] } },
  browser_implementation: 'Existing local Chrome through existing bundled Playwright; new isolated context',
  pages: [], page_errors: [], console_errors: [], failed_requests: [], nonlocal_requests: [], screenshots: [],
};
let browser;
function save() { fs.writeFileSync(path.join(out, 'interaction.json'), JSON.stringify(report, null, 2) + '\n'); }
function requireTrue(value, message) { if (!value) throw Error(message); }
function snapshotSources() {
  const index = read(path.join(gallery, 'index.json'));
  const digests = {};
  for (const row of index.illustrations) {
    const attempt = row.source_attempt;
    const receipt = read(path.join(attempt, 'receipt.json'));
    digests[path.join(attempt, 'receipt.json')] = fileHash(path.join(attempt, 'receipt.json'));
    for (const [name, expected] of Object.entries(receipt.files)) {
      const file = path.join(attempt, name);
      const actual = fileHash(file);
      requireTrue(actual === expected, 'Source receipt mismatch before/after browser check: ' + file);
      digests[file] = actual;
    }
  }
  return digests;
}
async function screenshot(page, name, locator) {
  const file = path.join(out, name + '.png');
  const bytes = locator ? await locator.screenshot({ path: file }) : await page.screenshot({ path: file, fullPage: false });
  report.screenshots.push({ name, file, sha256: hash(bytes), bytes: bytes.length });
  return hash(bytes);
}
async function exercise(page, name, seedOnly) {
  await page.waitForLoadState('networkidle');
  const entry = { name, url: page.url(), seed_only: seedOnly, passed: false };
  report.pages.push(entry); save();
  const button = page.locator('#play');
  const slider = page.locator('#slider');
  const stamp = page.locator('#stamp');
  const canvas = page.locator('#view');
  await button.waitFor({ state: 'visible' });
  entry.initial_stamp = await stamp.innerText();
  entry.initial_slider = await slider.inputValue();
  entry.frame_max_index = Number(await slider.getAttribute('max'));
  requireTrue(entry.frame_max_index > 0, 'Saved replay has no frame transition');
  entry.banner = await page.locator('p.card.status').innerText();
  entry.subtitle = await page.locator('#subtitle').innerText();
  requireTrue(seedOnly ? entry.banner.includes('SEED ONLY') && entry.subtitle.includes('collision_failed')
                      : entry.banner.includes('FINAL VALIDATED TRAJECTORY') && entry.subtitle.includes('success_validated_numerical'),
              'Final/seed-only status banner mismatch');
  entry.initial_canvas_sha256 = await screenshot(page, name + '_initial_canvas', canvas);
  await screenshot(page, name + '_initial');
  await button.click();
  await page.waitForFunction(initial => document.querySelector('#stamp').textContent !== initial, entry.initial_stamp);
  entry.playing_button = await button.innerText();
  entry.playing_stamp = await stamp.innerText();
  requireTrue(entry.playing_button === 'Pause', 'Play did not switch to Pause');
  await button.click();
  entry.paused_stamp = await stamp.innerText();
  requireTrue(await button.innerText() === 'Play', 'Pause did not return to Play');
  await page.waitForTimeout(350);
  entry.pause_after_350ms = await stamp.innerText();
  requireTrue(entry.paused_stamp === entry.pause_after_350ms, 'Frame kept advancing while paused');
  await slider.focus();
  await slider.press('End');
  await page.waitForFunction(max => Number(document.querySelector('#slider').value) === max, entry.frame_max_index);
  entry.end_slider = await slider.inputValue();
  entry.end_stamp = await stamp.innerText();
  requireTrue(entry.end_stamp.startsWith(String(entry.frame_max_index + 1) + ' / '), 'Frame counter did not reach the last frame');
  entry.end_canvas_sha256 = await screenshot(page, name + '_last_canvas', canvas);
  await screenshot(page, name + '_last');
  requireTrue(entry.initial_canvas_sha256 !== entry.end_canvas_sha256, 'Canvas image did not change across saved frames');
  await slider.press('Home');
  await page.waitForFunction(() => document.querySelector('#slider').value === '0');
  entry.home_stamp = await stamp.innerText();
  requireTrue(entry.home_stamp === entry.initial_stamp, 'Home did not restore the first saved frame');
  await page.locator('#focus').click();
  entry.view_toggle_label = await page.locator('#focus').innerText();
  requireTrue(entry.view_toggle_label === 'Focus robot', 'Full-scene view toggle did not update');
  entry.passed = true; save();
}

(async () => {
  try {
    const before = snapshotSources();
    report.original_source_files_before = before;
    browser = await chromium.launch({ headless: true, executablePath: '/usr/bin/google-chrome' });
    report.browser_version = await browser.version();
    const context = await browser.newContext({ viewport: { width: 1280, height: 1000 }, deviceScaleFactor: 1 });
    await context.route('**/*', route => {
      const url = route.request().url();
      if (url.startsWith(origin + '/')) return route.continue();
      report.nonlocal_requests.push(url); return route.abort();
    });
    const page = await context.newPage();
    page.on('pageerror', error => { report.page_errors.push(String(error)); save(); });
    page.on('console', message => { if (message.type() === 'error') { report.console_errors.push(message.text()); save(); } });
    page.on('requestfailed', request => { report.failed_requests.push({ url: request.url(), reason: request.failure() }); save(); });
    const response = await page.goto(origin + '/index.html', { waitUntil: 'networkidle' });
    requireTrue(response.status() === 200, 'Gallery HTTP response was not 200');
    report.gallery = { url: page.url(), status: response.status(), links: await page.locator('a').count() };
    requireTrue(report.gallery.links === 8, 'Gallery does not expose all eight replay links');
    await screenshot(page, 'gallery');
    await page.locator('a[href="EI_ZO_distance_blocked/replay.html"]').click();
    await exercise(page, 'success_blocked', false);
    await page.goto(origin + '/index.html', { waitUntil: 'networkidle' });
    await page.locator('a[href="failure_seed_only/replay.html"]').click();
    await exercise(page, 'failure_seed_only', true);
    requireTrue(report.page_errors.length === 0, 'JavaScript page errors occurred');
    requireTrue(report.failed_requests.length === 0, 'Page requests failed');
    requireTrue(report.nonlocal_requests.length === 0, 'Page tried to contact nonlocal resources');
    const after = snapshotSources();
    requireTrue(JSON.stringify(before) === JSON.stringify(after), 'Original saved evidence changed during browser interaction');
    report.original_source_files_unchanged = true;
    report.original_source_file_count = Object.keys(before).length;
    report.passed = true;
  } catch (error) {
    report.error = error.stack || String(error);
    process.exitCode = 1;
  } finally {
    if (browser) { await browser.close(); report.browser_closed = true; }
    report.finished_utc = new Date().toISOString();
    save();
    console.log(JSON.stringify({ passed: report.passed, pages: report.pages, page_errors: report.page_errors,
      console_errors: report.console_errors, failed_requests: report.failed_requests,
      nonlocal_requests: report.nonlocal_requests, original_source_file_count: report.original_source_file_count,
      original_source_files_unchanged: report.original_source_files_unchanged, error: report.error,
      output: out, browser_closed: report.browser_closed }, null, 2));
  }
})();
