import{_ as a,o as n,c as i,a2 as e}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse('{"title":"auditTime - laatste waarde afgegeven na opgegeven tijd","description":"auditTime is een RxJS filteroperator die wacht op een gespecificeerde tijd wanneer een waarde wordt afgegeven en de laatste waarde binnen die periode uitvoert. Het wordt het best gebruikt als je periodiek de laatste status wilt bemonsteren op hoogfrequente gebeurtenissen zoals scrollpositie bijhouden, venster verkleinen, muisbeweging, enz. Het is belangrijk om het verschil te begrijpen tussen dit en throttleTime en debounceTime en ze op de juiste manier te gebruiken.","frontmatter":{"description":"auditTime is een RxJS filteroperator die wacht op een gespecificeerde tijd wanneer een waarde wordt afgegeven en de laatste waarde binnen die periode uitvoert. Het wordt het best gebruikt als je periodiek de laatste status wilt bemonsteren op hoogfrequente gebeurtenissen zoals scrollpositie bijhouden, venster verkleinen, muisbeweging, enz. Het is belangrijk om het verschil te begrijpen tussen dit en throttleTime en debounceTime en ze op de juiste manier te gebruiken."},"headers":[],"relativePath":"nl/guide/operators/filtering/auditTime.md","filePath":"nl/guide/operators/filtering/auditTime.md","lastUpdated":1779063978000}'),p={name:"nl/guide/operators/filtering/auditTime.md"};function t(l,s,h,k,r,o){return n(),i("div",null,[...s[0]||(s[0]=[e(`<h1 id="audittime-laatste-waarde-afgegeven-na-opgegeven-tijd" tabindex="-1">auditTime - laatste waarde afgegeven na opgegeven tijd <a class="header-anchor" href="#audittime-laatste-waarde-afgegeven-na-opgegeven-tijd" aria-label="Permalink to &quot;auditTime - laatste waarde afgegeven na opgegeven tijd&quot;">​</a></h1><p>De <code>auditTime</code> operator wacht tot een <strong>gespecificeerde tijd</strong> nadat een waarde is uitgegeven en voert de <strong>laatste waarde</strong> binnen die periode uit. Daarna wordt gewacht op de volgende waarde.</p><h2 id="🔰-basis-syntaxis-en-gebruik" tabindex="-1">🔰 Basis syntaxis en gebruik <a class="header-anchor" href="#🔰-basis-syntaxis-en-gebruik" aria-label="Permalink to &quot;🔰 Basis syntaxis en gebruik&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Klik.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span></code></pre></div><p><strong>Bedieningsstroom</strong>:.</p><ol><li>eerste klik vindt plaats</li><li>wacht 1 seconde (klikken gedurende deze tijd worden geregistreerd, maar niet uitgevoerd)</li><li>de laatste klik wordt na 1 seconde uitgevoerd</li><li>wacht op de volgende klik</li></ol><p><a href="https://rxjs.dev/api/operators/auditTime" target="_blank" rel="noreferrer">🌐 RxJS officiële documentatie - auditTime</a></p><h2 id="🆚-contrast-met-throttletime" tabindex="-1">🆚 Contrast met throttleTime <a class="header-anchor" href="#🆚-contrast-met-throttletime" aria-label="Permalink to &quot;🆚 Contrast met throttleTime&quot;">​</a></h2><p><code>throttleTime</code> en <code>auditTime</code> lijken op elkaar, maar verschillen in de waarden die ze uitvoeren.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { throttleTime, auditTime, take } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">300</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">take</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// throttleTime: Eerste waarde uitvoeren</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  throttleTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitvoeren.: 0, 4, 8(eerste waarde van elke periode)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// auditTime: Uitvoer laatste waarde</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitvoeren.: 3, 6, 9(laatste waarde van elke periode)</span></span></code></pre></div><p><strong>Tijdlijnvergelijking</strong>:.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>Bron:     0--1--2--3--4--5--6--7--8--9--|</span></span>
<span class="line"><span>            |        |        |</span></span>
<span class="line"><span>throttle:   0--------4--------8------------|</span></span>
<span class="line"><span>            (Eerste)   (Eerste)   (Eerste)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>audit:      -------3--------6--------9----|</span></span>
<span class="line"><span>                  (Laatste)   (Laatste)   (Laatste)</span></span></code></pre></div><p>TABEL 12</p><h2 id="💡-typisch-gebruikspatroon" tabindex="-1">💡 Typisch gebruikspatroon <a class="header-anchor" href="#💡-typisch-gebruikspatroon" aria-label="Permalink to &quot;💡 Typisch gebruikspatroon&quot;">​</a></h2><ol><li><strong>Het formaat van het venster aanpassen</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;resize&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">200</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">) </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 200msVerkrijg de laatste grootte in het interval</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Venstergrootte: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerWidth</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}x\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerHeight</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span></code></pre></div><ol start="2"><li><p><strong>Scrollpositie bijhouden</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;scroll&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">100</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  map</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ({</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollY: window.scrollY,</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollX: window.scrollX</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">position</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Scrollpositie: Y=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollY</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}, X=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollX</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li><li><p><strong>Vloeiende sleepbeweging</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map, takeUntil, switchMap } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sleepbare elementen maken</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> box</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;div&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.width </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.height </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.backgroundColor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;#3498db&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.position </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;absolute&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.cursor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;move&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.left </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.top </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Slepen&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.display </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;flex&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.alignItems </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;center&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.justifyContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;center&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.color </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;white&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(box);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> mouseDown$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">MouseEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;(box, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;mousedown&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> mouseMove$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">MouseEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;mousemove&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> mouseUp$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">MouseEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;mouseup&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Slepen implementeren</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">mouseDown$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  switchMap</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">startEvent</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startX</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientX </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetLeft;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startY</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientY </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetTop;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    return</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> mouseMove$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">      auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">16</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">), </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ongeveer.60FPS(zie ook16ms) om de positie bij te werken</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">      map</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">moveEvent</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ({</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">        x: moveEvent.clientX </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startX,</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">        y: moveEvent.clientY </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startY</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">      })),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">      takeUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(mouseUp$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    );</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  })</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">position</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  box.style.left </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> \`\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">x</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}px\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  box.style.top </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> \`\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">y</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}px\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li></ol><h2 id="🧠-praktisch-codevoorbeeld-muis-volgen" tabindex="-1">🧠 Praktisch codevoorbeeld (muis volgen) <a class="header-anchor" href="#🧠-praktisch-codevoorbeeld-muis-volgen" aria-label="Permalink to &quot;🧠 Praktisch codevoorbeeld (muis volgen)&quot;">​</a></h2><p>Dit voorbeeld houdt muisbewegingen bij en geeft regelmatig de laatste positie weer.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// UI-elementen maken</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>container.style.height = &#39;300px&#39;;</span></span>
<span class="line"><span>container.style.border = &#39;2px solid #3498db&#39;;</span></span>
<span class="line"><span>container.style.padding = &#39;20px&#39;;</span></span>
<span class="line"><span>container.style.position = &#39;relative&#39;;</span></span>
<span class="line"><span>container.textContent = &#39;Beweeg de muis binnen dit gebied&#39;;</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const positionDisplay = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>positionDisplay.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>positionDisplay.style.fontFamily = &#39;monospace&#39;;</span></span>
<span class="line"><span>document.body.appendChild(positionDisplay);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const dot = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>dot.style.width = &#39;10px&#39;;</span></span>
<span class="line"><span>dot.style.height = &#39;10px&#39;;</span></span>
<span class="line"><span>dot.style.borderRadius = &#39;50%&#39;;</span></span>
<span class="line"><span>dot.style.backgroundColor = &#39;#e74c3c&#39;;</span></span>
<span class="line"><span>dot.style.position = &#39;absoluut&#39;;</span></span>
<span class="line"><span>dot.style.display = &#39;none&#39;;</span></span>
<span class="line"><span>container.appendChild(dot);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Muisbewegingsgebeurtenis</span></span>
<span class="line"><span>fromEvent&lt;MouseEvent&gt;(container, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>  map(event =&gt; {</span></span>
<span class="line"><span>    const rect = container.getBoundingClientRect();</span></span>
<span class="line"><span>    return {</span></span>
<span class="line"><span>      x: event.clientX - rect.left,.</span></span>
<span class="line"><span>      y: event.clientY - rect.top</span></span>
<span class="line"><span>    };</span></span>
<span class="line"><span>  }),</span></span>
<span class="line"><span>  auditTime(100) // Elke 100 ms de laatste positie ophalen</span></span>
<span class="line"><span>).subscribe(position =&gt; {</span></span>
<span class="line"><span>  positionDisplay.textContent = \`Laatste positie (elke 100 ms): X=\${position.x.toFixed(0)}, Y=\${position.y.toFixed(0)}\`;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Verplaats dot naar laatste positie</span></span>
<span class="line"><span>  dot.style.left = \`\${position.x - 5}px\`;</span></span>
<span class="line"><span>  dot.style.top = \`\${position.y - 5}px\`;</span></span>
<span class="line"><span>  dot.style.display = &#39;block&#39;;</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Deze code zal alleen de laatste positie ophalen en weergeven telkens als de muis wordt verplaatst, zelfs als de muis vaak wordt verplaatst,100msDe code haalt alleen de laatste positie op en geeft deze weer bij elke muisbeweging.</p><h2 id="🎯-debouncetime-verschillen-tussen" tabindex="-1">🎯 debounceTime Verschillen tussen <a class="header-anchor" href="#🎯-debouncetime-verschillen-tussen" aria-label="Permalink to &quot;🎯 debounceTime Verschillen tussen&quot;">​</a></h2><p><code>auditTime</code> en <code>debounceTime</code> is dat<strong>beide de laatste waarde uitvoeren</strong>maar de<strong>De timing is compleet anders</strong>de laatste waarde wordt uitgevoerd.</p><h3 id="het-doorslaggevende-verschil" tabindex="-1">Het doorslaggevende verschil <a class="header-anchor" href="#het-doorslaggevende-verschil" aria-label="Permalink to &quot;Het doorslaggevende verschil&quot;">​</a></h3><table tabindex="0"><thead><tr><th>Operator</th><th>operatie</th><th>gebruik van het systeem op verschillende manieren</th></tr></thead><tbody><tr><td><code>auditTime(ms)</code></td><td>Wanneer een waarde binnenkomt<strong>msAltijd uitvoer na</strong>(zelfs als de invoer doorgaat)</td><td>Periodieke bemonstering</td></tr><tr><td><code>debounceTime(ms)</code></td><td><strong>Nadat de invoer is gestopt</strong>msUitvoeren na</td><td>Wachten op voltooiing van invoer</td></tr></tbody></table><h3 id="specifieke-voorbeelden-verschillen-in-zoekinvoer" tabindex="-1">Specifieke voorbeelden：Verschillen in zoekinvoer <a class="header-anchor" href="#specifieke-voorbeelden-verschillen-in-zoekinvoer" aria-label="Permalink to &quot;Specifieke voorbeelden：Verschillen in zoekinvoer&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, debounceTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.placeholder = &#39;Zoekwoord invoer&#39;;</span></span>
<span class="line"><span>document.body.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// auditTime: zoekactie elke 300 ms uitvoeren, zelfs tijdens invoer</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300)</span></span>
<span class="line"><span>).subscribe() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;auditTime → Zoeken:&#39;, input.value);</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// debounceTime: wacht 300ms nadat invoer stopt, voer dan zoeken uit</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300)</span></span>
<span class="line"><span>).subscribe() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;debounceTime → Zoeken:&#39;, input.value);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h3 id="verschillen-in-tijdlijn" tabindex="-1">Verschillen in tijdlijn <a class="header-anchor" href="#verschillen-in-tijdlijn" aria-label="Permalink to &quot;Verschillen in tijdlijn&quot;">​</a></h3><p>Verschil gezien wanneer een gebruiker klikt op &quot;ab&#39;→&#39;abc&#39;→&#39;abcd&#39; bij snel typen:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>Invoerevent: a-b--c--d------------|</span></span>
<span class="line"><span>              ↓</span></span>
<span class="line"><span>auditTime: ------c-----d----------|.</span></span>
<span class="line"><span>            (na 300 ms) (na 300 ms)</span></span>
<span class="line"><span>            → Zoek naar &#39;abc&#39;, zoek naar &#39;abcd&#39; (in totaal 2 keer)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>debounceTime: --------------------d-|</span></span>
<span class="line"><span>                              (300 ms na stop)</span></span>
<span class="line"><span>            → Zoek naar &#39;abcd&#39; (in totaal slechts één keer)</span></span></code></pre></div><p><strong>Gemakkelijk te onthouden</strong>:</p><ul><li><strong><code>auditTime</code></strong>: &#39;Regelmatig gecontroleerd (audit)&quot;→ &#39;Regelmatig controleren&#39;</li><li><strong><code>debounceTime</code></strong>: &#39;Wacht tot het rustig is (...)&#39;.debounceWacht tot het rustig is.→ &#39;Wacht tot het rustig is&#39;</li></ul><h3 id="praktisch-gebruik" tabindex="-1">Praktisch gebruik <a class="header-anchor" href="#praktisch-gebruik" aria-label="Permalink to &quot;Praktisch gebruik&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>// ✅ auditTime indien van toepassing</span></span>
<span class="line"><span>// - De scrollpositie bijhouden (we willen deze periodiek krijgen, zelfs als we de hele tijd scrollen)</span></span>
<span class="line"><span>fromEvent(window, &#39;scroll&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(100) // elke 100 ms de laatste positie krijgen</span></span>
<span class="line"><span>).subscribe(/* ... */);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ als debounceTime geschikt is.</span></span>
<span class="line"><span>// - zoekvak (we willen zoeken nadat de invoer is voltooid)</span></span>
<span class="line"><span>fromEvent(searchInput, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300) // wacht 300ms nadat invoer is gestopt</span></span>
<span class="line"><span>).subscribe(/* ... */);</span></span></code></pre></div><h2 id="📋-type-veilig-gebruik" tabindex="-1">📋 Type-veilig gebruik <a class="header-anchor" href="#📋-type-veilig-gebruik" aria-label="Permalink to &quot;📋 Type-veilig gebruik&quot;">​</a></h2><p>TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface MousePosition {</span></span>
<span class="line"><span>  x: getal;</span></span>
<span class="line"><span>  y: getal;</span></span>
<span class="line"><span>  timestamp: getal; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>functie trackMousePosition(</span></span>
<span class="line"><span>  element: HTMLElement,.</span></span>
<span class="line"><span>  interval: getal</span></span>
<span class="line"><span>): Observable {</span></span>
<span class="line"><span>  return fromEvent&lt;MouseEvent&gt;(element, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>    auditTime(intervalMs),.</span></span>
<span class="line"><span>    map(event =&gt; ({</span></span>
<span class="line"><span>      x: event.clientX, event.</span></span>
<span class="line"><span>      y: event.clientY,.</span></span>
<span class="line"><span>      timestamp: Date.now())</span></span>
<span class="line"><span>    } als MuisPositie))</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Gebruiksvoorbeeld</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>canvas.style.width = &#39;400px&#39;;</span></span>
<span class="line"><span>canvas.style.height = &#39;300px&#39;;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px solid black&#39;;</span></span>
<span class="line"><span>document.body.appendChild(canvas);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>trackMousePosition(canvas, 200).subscribe(position =&gt; {</span></span>
<span class="line"><span>  console.log(\`Positie: (\${position.x}, \${position.y}) op \${position.timestamp}\`);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h2 id="🔄-audittime-en-throttletime-combinatie-van" tabindex="-1">🔄 auditTime en throttleTime Combinatie van <a class="header-anchor" href="#🔄-audittime-en-throttletime-combinatie-van" aria-label="Permalink to &quot;🔄 auditTime en throttleTime Combinatie van&quot;">​</a></h2><p>In bepaalde scenario&#39;s kunnen beide worden gecombineerd.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { throttleTime, auditTime, take } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(100).pipe(take(50));.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// volgorde van throttleTime → auditTime</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  throttleTime(1000), // geef de eerste waarde elke seconde door</span></span>
<span class="line"><span>  auditTime(500) // wacht dan 500ms en voer de laatste waarde uit</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Klik.！&#39;));</span></span>
<span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Maak een invoerveld voor zoeken</span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);.</span></span>
<span class="line"><span>input.type = &#39;tekst&#39;;</span></span>
<span class="line"><span>input.placeholder = &quot;Zoeken... ;</span></span>
<span class="line"><span>document.body.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Slecht voorbeeld: gebruik auditTime voor zoekinvoer</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300) // zoeken wordt elke 300ms uitgevoerd tijdens het invoeren</span></span>
<span class="line"><span>).subscribe() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Zoekopdracht uitgevoerd&#39;);</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Klik.！&#39;));</span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Klik!&#39;));</span></span></code></pre></div><p>ts. import { fromEvent } from &#39;rxjs&#39;; import { debounceTime } from &#39;rxjs&#39;;</p><p>// Maak een zoekinvoerveld const input = document.createElement(&#39;input&#39;);. input.type = &#39;tekst&#39;; input.placeholder = &quot;Zoeken... ; document.body.appendChild(input);</p><p>// ✅ Goed voorbeeld: debounceTime gebruiken voor zoekinvoer fromEvent(input, &#39;input&#39;).pipe( debounceTime(300) // Wacht 300ms nadat invoer stopt voordat je gaat zoeken ).subscribe() =&gt; { console.log(&#39;Zoeken uitgevoerd&#39;, input.value); });</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>## Samenvatting</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Wanneer auditTime moet worden gebruikt.</span></span>
<span class="line"><span>- ✅ Wanneer regelmatig bijgewerkte waarden nodig zijn</span></span>
<span class="line"><span>- ✅ Hoogfrequente gebeurtenissen zoals scrollen, wijzigen van grootte, muisbeweging</span></span>
<span class="line"><span>- ✅ Wanneer periodieke bemonstering vereist is</span></span>
<span class="line"><span>- ✅ Wanneer je de laatste status wilt weergeven.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Wanneer throttleTime moet worden gebruikt.</span></span>
<span class="line"><span>- ✅ Als een onmiddellijke respons vereist is</span></span>
<span class="line"><span>- ✅ Als je de verwerking wilt starten met de eerste waarde</span></span>
<span class="line"><span>- ✅ Voorkomen van knoppen indrukken</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Wanneer debounceTime gebruiken.</span></span>
<span class="line"><span>- ✅ Als u wilt wachten op voltooiing van invoer</span></span>
<span class="line"><span>- ✅ Zoeken, automatisch aanvullen</span></span>
<span class="line"><span>- ✅ Wachten tot de gebruiker stopt met typen.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Opmerkingen.</span></span>
<span class="line"><span>- ⚠️ \`auditTime\` voert alleen de laatste waarde in de periode uit (tussenliggende waarden worden genegeerd)</span></span>
<span class="line"><span>- ⚠️ Niet erg effectief indien ingesteld voor korte intervallen</span></span>
<span class="line"><span>- ⚠️ \`throttleTime\` of \`debounceTime\` kunnen geschikter zijn, afhankelijk van de toepassing.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>## Volgende stappen.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>- **[throttleTime](. /throttleTime)** - leer hoe je de eerste waarde doorgeeft.</span></span>
<span class="line"><span>- **[debounceTime](. /debounceTime)** - leer hoe je waarden doorgeeft nadat de invoer stopt.</span></span>
<span class="line"><span>- Filter](. /filter)** - leer filteren op basis van voorwaarden.</span></span>
<span class="line"><span>- **[filtering-operator-praktische-gebruiksgevallen](. /practical-use-cases)** - leer echte use-cases gebruiken</span></span></code></pre></div>`,44)])])}const E=a(p,[["render",t]]);export{c as __pageData,E as default};
