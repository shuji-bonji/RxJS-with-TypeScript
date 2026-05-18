import{_ as n,o as i,c as a,a2 as e}from"./chunks/framework.B0tZAgFO.js";const E=JSON.parse('{"title":"auditTime - letzter Wert, der nach der angegebenen Zeit ausgegeben wurde","description":"auditTime ist ein RxJS-Filteroperator, der auf eine bestimmte Zeit wartet, wenn ein Wert ausgegeben wird, und den letzten Wert innerhalb dieses Zeitraums ausgibt. Er wird am besten verwendet, wenn Sie regelmäßig den letzten Zustand bei hochfrequenten Ereignissen wie der Verfolgung der Bildlaufposition, der Größenänderung des Fensters, der Mausbewegung usw. abfragen möchten. Es ist wichtig, den Unterschied zwischen diesem Operator und throttleTime und debounceTime zu verstehen und sie entsprechend zu verwenden.","frontmatter":{"description":"auditTime ist ein RxJS-Filteroperator, der auf eine bestimmte Zeit wartet, wenn ein Wert ausgegeben wird, und den letzten Wert innerhalb dieses Zeitraums ausgibt. Er wird am besten verwendet, wenn Sie regelmäßig den letzten Zustand bei hochfrequenten Ereignissen wie der Verfolgung der Bildlaufposition, der Größenänderung des Fensters, der Mausbewegung usw. abfragen möchten. Es ist wichtig, den Unterschied zwischen diesem Operator und throttleTime und debounceTime zu verstehen und sie entsprechend zu verwenden."},"headers":[],"relativePath":"de/guide/operators/filtering/auditTime.md","filePath":"de/guide/operators/filtering/auditTime.md","lastUpdated":1779063978000}'),p={name:"de/guide/operators/filtering/auditTime.md"};function t(l,s,h,k,r,d){return i(),a("div",null,[...s[0]||(s[0]=[e(`<h1 id="audittime-letzter-wert-der-nach-der-angegebenen-zeit-ausgegeben-wurde" tabindex="-1">auditTime - letzter Wert, der nach der angegebenen Zeit ausgegeben wurde <a class="header-anchor" href="#audittime-letzter-wert-der-nach-der-angegebenen-zeit-ausgegeben-wurde" aria-label="Permalink to &quot;auditTime - letzter Wert, der nach der angegebenen Zeit ausgegeben wurde&quot;">​</a></h1><p>Der Operator &quot;auditTime&quot; wartet auf eine <strong>angegebene Zeit</strong>, nachdem ein Wert ausgegeben wurde, und gibt den <strong>letzten Wert</strong> innerhalb dieses Zeitraums aus. Danach wartet er auf den nächsten Wert.</p><h2 id="🔰-grundlegende-syntax-und-verwendung" tabindex="-1">🔰 Grundlegende Syntax und Verwendung <a class="header-anchor" href="#🔰-grundlegende-syntax-und-verwendung" aria-label="Permalink to &quot;🔰 Grundlegende Syntax und Verwendung&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Anklicken.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span></code></pre></div><p><strong>Ablauf der Operation</strong>:.</p><ol><li>der erste Klick erfolgt</li><li>1 Sekunde warten (Klicks während dieser Zeit werden aufgezeichnet, aber nicht ausgegeben)</li><li>gibt den letzten Klick nach 1 Sekunde aus Warten auf den nächsten Klick</li></ol><p><a href="https://rxjs.dev/api/operators/auditTime" target="_blank" rel="noreferrer">🌐 RxJS offizielle Dokumentation - <code>auditTime</code></a></p><h2 id="🆚-gegensatz-zu-throttletime" tabindex="-1">🆚 Gegensatz zu throttleTime <a class="header-anchor" href="#🆚-gegensatz-zu-throttletime" aria-label="Permalink to &quot;🆚 Gegensatz zu throttleTime&quot;">​</a></h2><p><code>throttleTime</code> und <code>auditTime</code> sind ähnlich, unterscheiden sich aber in den Werten, die sie ausgeben.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { throttleTime, auditTime, take } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">300</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">take</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// throttleTime: Ersten Wert ausgeben</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  throttleTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgeben.: 0, 4, 8(erster Wert der jeweiligen Periode)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// auditTime: Letzten Wert ausgeben</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgeben.: 3, 6, 9(letzter Wert jeder Periode)</span></span></code></pre></div><p><strong>Zeitlinienvergleich</strong>:.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>Quelle:     0--1--2--3--4--5--6--7--8--9--|</span></span>
<span class="line"><span>            |        |        |</span></span>
<span class="line"><span>throttle:   0--------4--------8------------|</span></span>
<span class="line"><span>            (Erste)   (Erste)   (Erste)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>audit:      -------3--------6--------9----|</span></span>
<span class="line"><span>                  (Letzter)   (Letzter)   (Letzter)</span></span></code></pre></div><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Anklicken.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`\`\`ts</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">import { interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">import { throttleTime, auditTime, take } from &#39;rxjs&#39;;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// throttleTime: ErsteのWertをAusgabe</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">source$.pipe(</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">  throttleTime(1000)</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">).subscribe(console.log);</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// Ausgabe: 0, 4, 8（各PeriodeのErsteのWert）</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// auditTime: LetzteのWertをAusgabe</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">source$.pipe(</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">  auditTime(1000)</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">).subscribe(console.log);</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// Ausgabe: 3, 6, 9（各PeriodeのLetzteのWert）</span></span></code></pre></div><h2 id="💡-typisches-nutzungsmuster" tabindex="-1">💡 Typisches Nutzungsmuster <a class="header-anchor" href="#💡-typisches-nutzungsmuster" aria-label="Permalink to &quot;💡 Typisches Nutzungsmuster&quot;">​</a></h2><ol><li><strong>Optimierung der Fenstergröße</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;resize&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">200</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">) </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 200msAbfrage der letzten Größe im Intervall</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Größe des Fensters: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerWidth</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}x\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerHeight</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span></code></pre></div><ol start="2"><li><p><strong>Verfolgung der Bildlaufposition</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;scroll&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">100</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  map</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ({</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollY: window.scrollY,</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollX: window.scrollX</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">position</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Position des Bildlaufs: Y=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollY</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}, X=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollX</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li><li><p><strong>Sanfte Ziehbewegung</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map, takeUntil, switchMap } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ziehbare Elemente erstellen</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> box</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;div&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.width </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.height </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.backgroundColor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;#3498db&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.position </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;absolute&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.cursor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;move&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.left </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.top </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Ziehen&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Implementierung von Ziehoperationen</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">mouseDown$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  switchMap</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">startEvent</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startX</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientX </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetLeft;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startY</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientY </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetTop;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    return</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> mouseMove$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">      auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">16</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">), </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ungefähr.60FPS(siehe auch16ms) zur Aktualisierung der Position</span></span>
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
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li></ol><h2 id="🧠-praktisches-codebeispiel-mausverfolgung" tabindex="-1">🧠 Praktisches Codebeispiel (Mausverfolgung) <a class="header-anchor" href="#🧠-praktisches-codebeispiel-mausverfolgung" aria-label="Permalink to &quot;🧠 Praktisches Codebeispiel (Mausverfolgung)&quot;">​</a></h2><p>Dieses Beispiel verfolgt die Mausbewegungen und zeigt die letzte Position in regelmäßigen Abständen an.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { auditTime, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Erstellen von UI-Elementen</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>container.style.height = &#39;300px&#39;;</span></span>
<span class="line"><span>container.style.border = &#39;2px solid #3498db&#39;;</span></span>
<span class="line"><span>container.style.padding = &#39;20px&#39;;</span></span>
<span class="line"><span>container.style.position = &#39;relativ&#39;;</span></span>
<span class="line"><span>container.textContent = &#39;Bitte bewegen Sie die Maus innerhalb dieses Bereichs&#39;;</span></span>
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
<span class="line"><span>dot.style.position = &#39;absolut&#39;;</span></span>
<span class="line"><span>dot.style.display = &#39;none&#39;;</span></span>
<span class="line"><span>container.appendChild(dot);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Mausbewegungs-Ereignis</span></span>
<span class="line"><span>fromEvent&lt;MouseEvent&gt;(container, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>  map(event =&gt; {</span></span>
<span class="line"><span>    const rect = container.getBoundingClientRect();</span></span>
<span class="line"><span>    return {</span></span>
<span class="line"><span>      x: event.clientX - rect.left,.</span></span>
<span class="line"><span>      y: event.clientY - rect.top</span></span>
<span class="line"><span>    };</span></span>
<span class="line"><span>  }),</span></span>
<span class="line"><span>  auditTime(100) // Abrufen der neuesten Position alle 100ms</span></span>
<span class="line"><span>).subscribe(position =&gt; {</span></span>
<span class="line"><span>  positionDisplay.textContent = \`Letzte Position (alle 100ms): X=\${position.x.toFixed(0)}, Y=\${position.y.toFixed(0)}\`;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Punkt an die letzte Position verschieben</span></span>
<span class="line"><span>  dot.style.left = \`\${Position.x - 5}px\`;</span></span>
<span class="line"><span>  dot.style.top = \`\${Position.y - 5}px\`;</span></span>
<span class="line"><span>  dot.style.display = &#39;block&#39;;</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Dieser Code ruft nur bei jeder Mausbewegung die letzte Position ab und zeigt sie an, auch wenn die Maus häufig bewegt wird,100msDer Code ruft nur die letzte Position bei jeder Mausbewegung ab und zeigt sie an.</p><h2 id="🎯-debouncetime-unterschiede-zwischen" tabindex="-1">🎯 debounceTime Unterschiede zwischen <a class="header-anchor" href="#🎯-debouncetime-unterschiede-zwischen" aria-label="Permalink to &quot;🎯 debounceTime Unterschiede zwischen&quot;">​</a></h2><p><code>auditTime</code> und <code>debounceTime</code> ist, dass<strong>beide den letzten Wert ausgeben</strong>aber die<strong>Das Timing ist völlig unterschiedlich</strong>der letzte Wert ausgegeben wird.</p><h3 id="der-entscheidende-unterschied" tabindex="-1">Der entscheidende Unterschied <a class="header-anchor" href="#der-entscheidende-unterschied" aria-label="Permalink to &quot;Der entscheidende Unterschied&quot;">​</a></h3><table tabindex="0"><thead><tr><th>Bediener</th><th>Betrieb</th><th>die unterschiedliche Nutzung des Systems</th></tr></thead><tbody><tr><td><code>auditTime(ms)</code></td><td>Wenn ein Wert eintrifft<strong>msAusgabe immer nach</strong>(auch wenn die Eingabe fortgesetzt wird)</td><td>Periodische Abtastung</td></tr><tr><td><code>debounceTime(ms)</code></td><td><strong>Nachdem die Eingabe gestoppt wurde</strong>msAusgabe danach</td><td>Warten auf Abschluss der Eingabe</td></tr></tbody></table><h3 id="spezifische-beispiele-unterschiede-in-der-sucheingabe" tabindex="-1">Spezifische Beispiele：Unterschiede in der Sucheingabe <a class="header-anchor" href="#spezifische-beispiele-unterschiede-in-der-sucheingabe" aria-label="Permalink to &quot;Spezifische Beispiele：Unterschiede in der Sucheingabe&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, debounceTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.placeholder = &#39;Suchworteingabe&#39;;</span></span>
<span class="line"><span>document.body.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// auditTime: Suche auch während der Eingabe alle 300ms ausführen</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300)</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;auditTime → Suche:&#39;, input.value);</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// debounceTime: 300ms nach Ende der Eingabe warten, dann Suche ausführen</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300)</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;debounceTime → Suche:&#39;, input.value);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h3 id="unterschiede-in-der-zeitleiste" tabindex="-1">Unterschiede in der Zeitleiste <a class="header-anchor" href="#unterschiede-in-der-zeitleiste" aria-label="Permalink to &quot;Unterschiede in der Zeitleiste&quot;">​</a></h3><p>Unterschied, wenn ein Benutzer auf &quot;&quot; klicktab&#39;→&#39;abc&#39;→&#39;abcd&#39; beim schnellen Tippen:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>Eingabe-Ereignis: a--b--c--d------------|</span></span>
<span class="line"><span>              ↓</span></span>
<span class="line"><span>auditTime: ------c-----d----------|</span></span>
<span class="line"><span>            (nach 300 ms) (nach 300 ms)</span></span>
<span class="line"><span>            → Suche nach &#39;abc&#39;, Suche nach &#39;abcd&#39; (insgesamt 2 Mal)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>debounceTime: --------------------d-|</span></span>
<span class="line"><span>                              (300 ms nach Stopp)</span></span>
<span class="line"><span>            → Suche nach &quot;abcd&quot; (insgesamt nur einmal)</span></span></code></pre></div><p><strong>Leicht zu merken</strong>:</p><ul><li><strong><code>auditTime</code></strong>: &#39;Regelmäßig geprüft (audit)&quot;→ &#39;In regelmäßigen Abständen prüfen&#39;</li><li><strong><code>debounceTime</code></strong>: &#39;Warten Sie, bis es ruhig geworden ist (...)&#39;.debounceWarten Sie, bis es ruhig ist.→ &#39;Warten Sie, bis es ruhig ist&#39;</li></ul><h3 id="praktische-anwendung" tabindex="-1">Praktische Anwendung <a class="header-anchor" href="#praktische-anwendung" aria-label="Permalink to &quot;Praktische Anwendung&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>// ✅ auditTime falls erforderlich</span></span>
<span class="line"><span>// - Verfolgung der Scroll-Position (wir wollen sie regelmäßig erhalten, auch wenn wir die ganze Zeit scrollen)</span></span>
<span class="line"><span>fromEvent(window, &#39;scroll&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(100) // Abrufen der neuesten Position alle 100ms</span></span>
<span class="line"><span>).subscribe(/* ... */);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ wenn debounceTime angemessen ist.</span></span>
<span class="line"><span>// - Suchfeld (wir wollen nach Abschluss der Eingabe suchen)</span></span>
<span class="line"><span>fromEvent(searchInput, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300) // 300ms nach Ende der Eingabe warten</span></span>
<span class="line"><span>).subscribe(/* ... */);</span></span></code></pre></div><h2 id="📋-typsichere-verwendung" tabindex="-1">📋 Typsichere Verwendung <a class="header-anchor" href="#📋-typsichere-verwendung" aria-label="Permalink to &quot;📋 Typsichere Verwendung&quot;">​</a></h2><p>TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, die die Generika in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { auditTime, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface MousePosition {</span></span>
<span class="line"><span>  x: Zahl;</span></span>
<span class="line"><span>  y: Zahl;</span></span>
<span class="line"><span>  timestamp: Zahl; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function trackMousePosition(</span></span>
<span class="line"><span>  element: HTMLElement,.</span></span>
<span class="line"><span>  intervalMs: Zahl</span></span>
<span class="line"><span>): Observable {</span></span>
<span class="line"><span>  return fromEvent&lt;MouseEvent&gt;(element, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>    auditTime(intervalMs),.</span></span>
<span class="line"><span>    map(event =&gt; ({</span></span>
<span class="line"><span>      x: event.clientX, event.</span></span>
<span class="line"><span>      y: event.clientY,.</span></span>
<span class="line"><span>      timestamp: Date.now())</span></span>
<span class="line"><span>    } as MousePosition))</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Beispiel für die Verwendung</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>canvas.style.width = &#39;400px&#39;;</span></span>
<span class="line"><span>canvas.style.height = &#39;300px&#39;;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px solid black&#39;;</span></span>
<span class="line"><span>document.body.appendChild(canvas);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>trackMousePosition(canvas, 200).subscribe(position =&gt; {</span></span>
<span class="line"><span>  console.log(\`Position: (\${position.x}, \${position.y}) bei \${position.timestamp}\`);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h2 id="🔄-audittime-und-throttletime-kombination-von" tabindex="-1">🔄 auditTime und throttleTime Kombination von <a class="header-anchor" href="#🔄-audittime-und-throttletime-kombination-von" aria-label="Permalink to &quot;🔄 auditTime und throttleTime Kombination von&quot;">​</a></h2><p>In bestimmten Szenarien können beide kombiniert werden.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { throttleTime, auditTime, take } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(100).pipe(take(50));.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Reihenfolge von throttleTime → auditTime</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  throttleTime(1000), // den ersten Wert jede Sekunde durchgeben</span></span>
<span class="line"><span>  auditTime(500) // dann 500ms warten und den letzten Wert ausgeben</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Anklicken.！&#39;));</span></span>
<span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Ein Sucheingabefeld erstellen</span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);.</span></span>
<span class="line"><span>input.type = &#39;Text&#39;;</span></span>
<span class="line"><span>input.placeholder = &#39;Suche...&#39; ;</span></span>
<span class="line"><span>document.body.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Schlechtes Beispiel: auditTime für Sucheingabe verwenden</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300) // Suche wird alle 300ms während der Eingabe durchgeführt</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Suche ausgeführt&#39;);</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Anklicken.！&#39;));</span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Klick!&#39;));</span></span></code></pre></div><p>ts. import { fromEvent } from &#39;rxjs&#39;; import { debounceTime } from &#39;rxjs&#39;;</p><p>// Ein Sucheingabefeld erstellen const input = document.createElement(&#39;input&#39;);. input.type = &#39;Text&#39;; input.placeholder = &#39;Suche...&#39; ; document.body.appendChild(input);</p><p>// ✅ Gutes Beispiel: debounceTime für Sucheingabe verwenden fromEvent(input, &#39;input&#39;).pipe( debounceTime(300) // 300ms nach Ende der Eingabe warten, bevor gesucht wird ).subscribe(() =&gt; { console.log(&#39;Suche ausgeführt&#39;, input.value); });</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>## 🎓 Zusammenfassung</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Wann sollte auditTime verwendet werden.</span></span>
<span class="line"><span>- ✅ Wenn in regelmäßigen Abständen aktuelle Werte benötigt werden</span></span>
<span class="line"><span>- ✅ Hochfrequente Ereignisse wie Bildlauf, Größenänderung, Mausbewegung</span></span>
<span class="line"><span>- ✅ Wenn eine periodische Probenahme erforderlich ist</span></span>
<span class="line"><span>- ✅ Wenn Sie den neuesten Stand wiedergeben wollen.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Wenn throttleTime verwendet werden soll.</span></span>
<span class="line"><span>- ✅ Wenn eine sofortige Reaktion erforderlich ist</span></span>
<span class="line"><span>- ✅ Wenn die Verarbeitung mit dem ersten Wert beginnen soll</span></span>
<span class="line"><span>- ✅ Verhinderung von Tastenbetätigungen</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Wann sollte debounceTime verwendet werden.</span></span>
<span class="line"><span>- ✅ Wenn Sie auf den Abschluss der Eingabe warten wollen</span></span>
<span class="line"><span>- ✅ Suche, Autovervollständigen</span></span>
<span class="line"><span>- ✅ Warten Sie, bis der Benutzer aufhört zu tippen.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Hinweise.</span></span>
<span class="line"><span>- ⚠️ \`auditTime\` gibt nur den letzten Wert im Zeitraum aus (Zwischenwerte werden verworfen)</span></span>
<span class="line"><span>- ⚠️ Nicht sehr effektiv, wenn für kurze Intervalle eingestellt</span></span>
<span class="line"><span>- ⚠️ \`throttleTime\` oder \`debounceTime\` können je nach Anwendung besser geeignet sein</span></span>
<span class="line"><span></span></span>
<span class="line"><span>## 🚀 Nächste Schritte.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>- **[throttleTime](. /throttleTime)** - lernen Sie, wie man den ersten Wert weitergibt.</span></span>
<span class="line"><span>- **[debounceTime](. /debounceTime)** - lernen Sie, wie man Werte ausgibt, nachdem die Eingabe gestoppt wurde.</span></span>
<span class="line"><span>- **[filter](. /filter)** - lernen Sie, wie Sie auf der Grundlage von Bedingungen filtern können.</span></span>
<span class="line"><span>- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - lernen Sie, wie man reale Anwendungsfälle nutzt</span></span></code></pre></div>`,44)])])}const o=n(p,[["render",t]]);export{E as __pageData,o as default};
