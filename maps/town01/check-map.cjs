const {chromium}=require('/home/xieutopian/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{
const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
const page=await browser.newPage({viewport:{width:1450,height:1100}});
const errors=[];page.on('pageerror',e=>errors.push(String(e)));
await page.goto('file://'+process.cwd()+'/outputs/town01-map/town01-map.html');
await page.locator('#map').click({position:{x:400,y:400}});
let initial=JSON.parse(await page.locator('#json').inputValue());
await page.locator('#offset').fill('2');
let shifted=JSON.parse(await page.locator('#json').inputValue());
if(initial.spawn_point.x===shifted.spawn_point.x&&initial.spawn_point.y===shifted.spawn_point.y)throw Error('offset failed');
await page.locator('#mode').selectOption('route');
await page.locator('#direction').selectOption('180');
await page.locator('#map').screenshot({path:'outputs/town01-map/town01-coordinate-map.png'});
await page.screenshot({path:'outputs/town01-map/selector-preview.png',fullPage:true});
console.log(JSON.stringify({errors,initial,shifted,routeSelection:JSON.parse(await page.locator('#json').inputValue())}));
if(errors.length)throw Error(errors.join('\n'));
await browser.close();
})();
