import{_ as n,o as i,c as a,a2 as e}from"./chunks/framework.5Uf62z_5.js";const g=JSON.parse('{"title":"skipUntil - bis zur Zündung überspringen","description":"Der skipUntil-Operator überspringt alle Werte des ursprünglichen Observable, bis ein anderes Observable einen Wert ausgibt; danach wird der Wert wie üblich ausgegeben. Dies ist nützlich für zeitlich verzögerte Starts oder nachdem ein bestimmtes Ereignis eingetreten ist.","frontmatter":{"description":"Der skipUntil-Operator überspringt alle Werte des ursprünglichen Observable, bis ein anderes Observable einen Wert ausgibt; danach wird der Wert wie üblich ausgegeben. Dies ist nützlich für zeitlich verzögerte Starts oder nachdem ein bestimmtes Ereignis eingetreten ist."},"headers":[],"relativePath":"de/guide/operators/filtering/skipUntil.md","filePath":"de/guide/operators/filtering/skipUntil.md","lastUpdated":1779269732000}'),p={name:"de/guide/operators/filtering/skipUntil.md"};function t(l,s,h,k,r,d){return i(),a("div",null,[...s[0]||(s[0]=[e(`<h1 id="skipuntil-bis-zur-zundung-uberspringen" tabindex="-1">skipUntil - bis zur Zündung überspringen <a class="header-anchor" href="#skipuntil-bis-zur-zundung-uberspringen" aria-label="Permalink to &quot;skipUntil - bis zur Zündung überspringen&quot;">​</a></h1><p>Der Operator &quot;skipUntil&quot; <strong>überspringt alle Werte des ursprünglichen Observable</strong>, bis der erste Wert vom angegebenen Observable ausgegeben wird (Notification Trigger). Nach dem Zeitpunkt, an dem der Notification Trigger ausgegeben wird, werden die Werte wie gewohnt ausgegeben.</p><h2 id="🔰-grundlegende-syntax-und-verwendung" tabindex="-1">🔰 Grundlegende Syntax und Verwendung <a class="header-anchor" href="#🔰-grundlegende-syntax-und-verwendung" aria-label="Permalink to &quot;🔰 Grundlegende Syntax und Verwendung&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Wert jede Sekunde ausgeben</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Wert nach jeder Sekunde ausgeben</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// (erster2zweiter Wert 0, 1, 2, 3 werden übersprungen)</span></span></code></pre></div><p><strong>Ablauf der Operation</strong>:.</p><ol><li><code>source$</code> gibt 0, 1, 2, 3 aus → alles überspringen</li><li>2 Sekunden später gibt <code>notifier$</code> einen Wert aus</li><li>die folgenden <code>Quelle$</code>-Werte (4, 5, 6, ...) werden wie üblich ausgegeben.</li></ol><p><a href="https://rxjs.dev/api/operators/skipUntil" target="_blank" rel="noreferrer">🌐 Offizielle RxJS Dokumentation - <code>skipUntil</code></a></p><h2 id="🆚-gegensatz-zu-takeuntil" tabindex="-1">🆚 Gegensatz zu takeUntil <a class="header-anchor" href="#🆚-gegensatz-zu-takeuntil" aria-label="Permalink to &quot;🆚 Gegensatz zu takeUntil&quot;">​</a></h2><p>skipUntil&quot; und &quot;takeUntil&quot; haben ein gegensätzliches Verhalten.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil, takeUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Wert jede Sekunde ausgeben</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Wert nach jeder Sekunde ausgeben</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// takeUntil: Abrufen des Wertes bis zur Benachrichtigung</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  takeUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe: 0, 1, 2, 3(Stoppt nach2(hält nach 1,5 Sekunden an)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// skipUntil: Überspringen von Werten bis zur Benachrichtigung</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe: 4, 5, 6, 7, ...(Stoppt nach2(Beginnt nach 1,5 Sekunden)</span></span></code></pre></div><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Wert jede Sekunde ausgeben</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Wert nach jeder Sekunde ausgeben</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// (erster2zweiter Wert 0, 1, 2, 3 werden übersprungen)</span></span></code></pre></div><h2 id="💡-typisches-nutzungsmuster" tabindex="-1">💡 Typisches Nutzungsmuster <a class="header-anchor" href="#💡-typisches-nutzungsmuster" aria-label="Permalink to &quot;💡 Typisches Nutzungsmuster&quot;">​</a></h2><ol><li><strong>Start der Datenverarbeitung nach Benutzerauthentifizierung</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, Subject } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> authenticated$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> new</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Subject</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">void</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;();</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> dataStream$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Daten überspringen, bis die Authentifizierung abgeschlossen ist</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   dataStream$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(authenticated$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">data</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Verarbeitung der Daten: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">data</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // 3Authentifizierung abgeschlossen nach 2 Sekunden</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   setTimeout</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Authentifizierung abgeschlossen！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     authenticated$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">next</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">();</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   }, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // 3(Beginn nach einer Sekunde) &quot;Datenverarbeitung: 3Datenverarbeitung: 4&#39;, &#39;Datenverarbeitung&#39;...und Ausgabe</span></span></code></pre></div><ol start="2"><li><p><strong>Ereignisverarbeitung beginnt nach Abschluss des ersten Ladens</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent, BehaviorSubject } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { filter, skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> appReady$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> new</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> BehaviorSubject</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">boolean</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">false</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> button</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;button&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">button.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Klicken.&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> clicks$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Klicks ignorieren, bis die App fertig ist</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">clicks$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(appReady$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">filter</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">ready</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ready)))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Klick verarbeitet&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2App bereit in Sekunden</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">setTimeout</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;App ist bereit&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  appReady$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">next</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">true</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span></code></pre></div></li><li><p><strong>Timer-basierte Verzögerung gestartet</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil, scan } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> button</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;button&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">button.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Zählung&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> clicks$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startTime$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 3Sekunden später</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 3Klicks werden erst nach Ablauf der Sekunden gezählt</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">clicks$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(startTime$),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  scan</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">count</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> count </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">+</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">count</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Zählung: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">count</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;3Zählung beginnt nach Sekunden...&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span></code></pre></div></li></ol><h2 id="🧠-praktisches-code-beispiel-spiel-countdown" tabindex="-1">🧠 Praktisches Code-Beispiel (Spiel-Countdown) <a class="header-anchor" href="#🧠-praktisches-code-beispiel-spiel-countdown" aria-label="Permalink to &quot;🧠 Praktisches Code-Beispiel (Spiel-Countdown)&quot;">​</a></h2><p>Dies ist ein Beispiel für das Ignorieren von Klicks während des Countdowns vor Beginn des Spiels und das Aktivieren von Klicks nach Ende des Countdowns.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent, timer, interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { skipUntil, take, scan } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Erstellen von UI-Elementen</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const count = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>countdown.style.fontSize = &#39;24px&#39;;</span></span>
<span class="line"><span>countdown.style.marginBottom = &#39;10px&#39;;</span></span>
<span class="line"><span>countdown.textContent = &#39;Countdown läuft...&#39; ;</span></span>
<span class="line"><span>container.appendChild(countdown);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const button = document.createElement(&#39;button&#39;);</span></span>
<span class="line"><span>button.textContent = &#39;Klick!&#39; ;</span></span>
<span class="line"><span>button.disabled = true;</span></span>
<span class="line"><span>container.appendChild(button);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const scoreDisplay = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>scoreDisplay.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>scoreDisplay.textContent = &#39;score: 0&#39;;</span></span>
<span class="line"><span>container.appendChild(scoreDisplay);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Countdown (3 Sekunden)</span></span>
<span class="line"><span>const countdownTimer$ = interval(1000).pipe(take(3));</span></span>
<span class="line"><span>countdownTimer$.subscribe({</span></span>
<span class="line"><span>  next: (n) =&gt; {</span></span>
<span class="line"><span>    countdown.textContent = \`\${3 - n} Sekunden bis zum Start... \`;</span></span>
<span class="line"><span>  },.</span></span>
<span class="line"><span>  complete: () =&gt; {</span></span>
<span class="line"><span>    countdown.textContent = &#39;Das Spiel beginnt!&#39; ;</span></span>
<span class="line"><span>    button.disabled = false;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Spielstart-Benachrichtigung</span></span>
<span class="line"><span>const gameStart$ = timer(3000);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Klick-Ereignis (springt zum Spielstart)</span></span>
<span class="line"><span>const clicks$ = fromEvent(button, &#39;click&#39;);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>clicks$.pipe(</span></span>
<span class="line"><span>  skipUntil(gameStart$),.</span></span>
<span class="line"><span>  scan(Spielstand =&gt; Spielstand + 10, 0)</span></span>
<span class="line"><span>).subscribe(Spielstand =&gt; {</span></span>
<span class="line"><span>  scoreDisplay.textContent = \`Punktestand: \${Punktestand}\`;</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>In diesem Code wird der Countdown3Sekunden, Klicks werden während des Countdowns ignoriert, und nur Klicks nach dem Ende des Countdowns werden im Ergebnis berücksichtigt.</p><h2 id="🎯-skip-der-unterschied-zwischen-skipuntil-unterschied-zwischen" tabindex="-1">🎯 skip Der Unterschied zwischen skipUntil Unterschied zwischen <a class="header-anchor" href="#🎯-skip-der-unterschied-zwischen-skipuntil-unterschied-zwischen" aria-label="Permalink to &quot;🎯 skip Der Unterschied zwischen skipUntil Unterschied zwischen&quot;">​</a></h2><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { skip, skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// skip: Überspringen des ersten N nach Nummer</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skip(3)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// Ausgabe: 3, 4, 5, 6, ...</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// skipUntil: Überspringen, bis ein anderes Observable ausgelöst wird</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(timer(1500))</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Ausgabe: 3, 4, 5, 6, ... (gleiches Ergebnis, aber andere Kontrollmethode)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Bediener</th><th>Bedingungen überspringen</th><th>Anwendungsfall</th></tr></thead><tbody><tr><td><code>skip(n)</code></td><td>ErstenÜberspringen einer bestimmten Anzahl von Teilen</td><td>Überspringen einer festen Anzahl</td></tr><tr><td><code>skipWhile(predicate)</code></td><td>Überspringen, wenn Bedingungen erfüllt sind</td><td>Bedingungsbasiertes Überspringen</td></tr><tr><td><code>skipUntil(notifier$)</code></td><td>Überspringen bis zum nächstenObservableÜberspringen, bis ein</td><td>Ereignis/Zeitbasiertes Überspringen</td></tr></tbody></table><h2 id="📋-typsichere-verwendung" tabindex="-1">📋 Typsichere Verwendung <a class="header-anchor" href="#📋-typsichere-verwendung" aria-label="Permalink to &quot;📋 Typsichere Verwendung&quot;">​</a></h2><p>TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, bei der Generika verwendet werden</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, Subject, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { skipUntil, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface GameState {</span></span>
<span class="line"><span>  status: &#39;waiting&#39; | &#39;ready&#39; | &#39;playing&#39; | &#39;finished&#39;;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface ClickEvent {</span></span>
<span class="line"><span>  timestamp: Zahl; }</span></span>
<span class="line"><span>  x: Zahl;</span></span>
<span class="line"><span>  y: Zahl;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>Klasse Game {</span></span>
<span class="line"><span>  private gameReady$ = new Subject();</span></span>
<span class="line"><span>  private state: GameState = { status: &#39;waiting&#39; };.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  startGame(element: HTMLElement): Observable {</span></span>
<span class="line"><span>    const clicks$ = fromEvent\\&lt;MouseEvent&gt;(element, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>      map(event =&gt; ({</span></span>
<span class="line"><span>        timestamp: Date.now(),.</span></span>
<span class="line"><span>        x: event.clientX, event.</span></span>
<span class="line"><span>        y: event.clientY</span></span>
<span class="line"><span>      } as ClickEvent))),.</span></span>
<span class="line"><span>      skipUntil(this.gameReady$)</span></span>
<span class="line"><span>    );</span></span>
<span class="line"><span></span></span>
<span class="line"><span>    // Benachrichtigung über die Bereitschaft</span></span>
<span class="line"><span>    setTimeout(() =&gt; {</span></span>
<span class="line"><span>      this.state = { status: &#39;ready&#39; };</span></span>
<span class="line"><span>      this.gameReady$.next();</span></span>
<span class="line"><span>      console.log(&#39;Spiel bereit!&#39;) ;</span></span>
<span class="line"><span>    }, 2000);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>    return clicks$;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Beispiel für die Verwendung</span></span>
<span class="line"><span>const game = new Game();</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>canvas.style.width = &#39;300px&#39;;</span></span>
<span class="line"><span>canvas.style.height = &#39;200px&#39;;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px solid black&#39;;</span></span>
<span class="line"><span>canvas.textContent = &#39;Hier klicken&#39;;</span></span>
<span class="line"><span>document.body.appendChild(canvas);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>game.startGame(canvas).subscribe(click =&gt; {</span></span>
<span class="line"><span>  console.log(\`Klickposition: (\${click.x}, \${click.y})\`);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h2 id="🔄-skipuntil-der-unterschied-zwischen-takeuntil-kombination-von" tabindex="-1">🔄 skipUntil Der Unterschied zwischen takeUntil Kombination von <a class="header-anchor" href="#🔄-skipuntil-der-unterschied-zwischen-takeuntil-kombination-von" aria-label="Permalink to &quot;🔄 skipUntil Der Unterschied zwischen takeUntil Kombination von&quot;">​</a></h2><p>Kombinieren Sie beides, wenn Sie nur Werte für einen bestimmten Zeitraum erhalten möchten.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, takeUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500);</span></span>
<span class="line"><span>const start$ = timer(2000); // Start nach 2 Sekunden</span></span>
<span class="line"><span>const stop$ = timer(5000); // stoppt nach 5 Sekunden</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(start$), // Überspringen bis nach 2 Sekunden</span></span>
<span class="line"><span>  takeUntil(stop$); // nach 5 Sekunden anhalten</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: 4, 5, 6, 7, 8, 9, complete.</span></span>
<span class="line"><span>// (es werden nur Werte zwischen 2 und 5 Sekunden abgerufen)</span></span></code></pre></div><p><strong>Zeitleisten</strong>:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>0s 1s 2s 3s 4s 5s</span></span>
<span class="line"><span></span></span>
<span class="line"><span>___TABELLE_11___</span></span>
<span class="line"><span></span></span>
<span class="line"><span>0 1 2 3 4 5 6 7 8 9 10</span></span>
<span class="line"><span>      ↑ hoch hoch hoch hoch hoch hoch</span></span>
<span class="line"><span>   SKIP Anfang TAKE Ende</span></span>
<span class="line"><span>   (von 4) (bis 9)</span></span></code></pre></div><h2 id="⚠️-ein-haufiger-fehler" tabindex="-1">⚠️ Ein häufiger Fehler <a class="header-anchor" href="#⚠️-ein-haufiger-fehler" aria-label="Permalink to &quot;⚠️ Ein häufiger Fehler&quot;">​</a></h2><div class="important custom-block github-alert"><p class="custom-block-title">IMPORTANT</p><p><code>skipUntil</code> sind Benachrichtigungen Observable der<strong>Nur das erste Feuern</strong>ist gültig.2Der zweite und die folgenden Abbrände werden ignoriert.</p></div><h3 id="falsch-benachrichtigungobservablewird-mehr-als-einmal-abgefeuert" tabindex="-1">Falsch: BenachrichtigungObservablewird mehr als einmal abgefeuert. <a class="header-anchor" href="#falsch-benachrichtigungobservablewird-mehr-als-einmal-abgefeuert" aria-label="Permalink to &quot;Falsch: BenachrichtigungObservablewird mehr als einmal abgefeuert.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { interval, Subject } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500);</span></span>
<span class="line"><span>const notifier$ = new Subject();</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(notifier$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Schlechtes Beispiel: next mehrfach aufrufen, aber nur das erste Mal hat eine Wirkung</span></span>
<span class="line"><span>setTimeout(() =&gt; notifier$.next(), 1000);</span></span>
<span class="line"><span>setTimeout(() =&gt; notifier$.next(), 2000); // dies ist sinnlos</span></span></code></pre></div><h3 id="richtig-nur-der-erste-abschuss-ist-gultig" tabindex="-1">Richtig.: Nur der erste Abschuss ist gültig. <a class="header-anchor" href="#richtig-nur-der-erste-abschuss-ist-gultig" aria-label="Permalink to &quot;Richtig.: Nur der erste Abschuss ist gültig.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, Subject } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importieren { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500);</span></span>
<span class="line"><span>const notifier$ = new Subject();</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(notifier$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Gutes Beispiel: next nur einmal aufrufen</span></span>
<span class="line"><span>setTimeout(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Ende des Überspringens&#39;);</span></span>
<span class="line"><span>  notifier$.next();</span></span>
<span class="line"><span>  notifier$.complete(); // best practice to complete.</span></span>
<span class="line"><span>}, 1000);</span></span></code></pre></div><h2 id="🎓-zusammenfassung" tabindex="-1">🎓 Zusammenfassung <a class="header-anchor" href="#🎓-zusammenfassung" aria-label="Permalink to &quot;🎓 Zusammenfassung&quot;">​</a></h2><h3 id="wann-sollte-skipuntil-verwendet-werden" tabindex="-1">Wann sollte skipUntil verwendet werden. <a class="header-anchor" href="#wann-sollte-skipuntil-verwendet-werden" aria-label="Permalink to &quot;Wann sollte skipUntil verwendet werden.&quot;">​</a></h3><ul><li>✅ Wenn Sie die Verarbeitung nach Eintreten eines bestimmten Ereignisses starten wollen</li><li>✅ Wenn Sie Benutzeroperationen nach Abschluss der Initialisierung ermöglichen wollen</li><li>✅ Wenn Sie einen zeitlich verzögerten Start benötigen</li><li>✅ Wenn Sie die Datenverarbeitung erst nach Abschluss der Authentifizierung starten wollen</li></ul><h3 id="in-kombination-mit-takeuntil" tabindex="-1">In Kombination mit takeUntil. <a class="header-anchor" href="#in-kombination-mit-takeuntil" aria-label="Permalink to &quot;In Kombination mit takeUntil.&quot;">​</a></h3><ul><li>✅ Wenn Sie Werte nur für einen bestimmten Zeitraum abrufen wollen (skipUntil + takeUntil)</li></ul><h3 id="hinweise" tabindex="-1">Hinweise. <a class="header-anchor" href="#hinweise" aria-label="Permalink to &quot;Hinweise.&quot;">​</a></h3><ul><li>⚠️ Nur das erste Feuern des Observable ist gültig</li><li>⚠️ Wenn das Observable nicht ausgelöst wird, werden weiterhin alle Werte übersprungen</li><li>⚠️ Das Abonnement wird aufrechterhalten, bis der ursprüngliche Stream abgeschlossen ist.</li></ul><h2 id="🚀-nachste-schritte" tabindex="-1">🚀 Nächste Schritte. <a class="header-anchor" href="#🚀-nachste-schritte" aria-label="Permalink to &quot;🚀 Nächste Schritte.&quot;">​</a></h2><ul><li><strong><a href="./skip">skip</a></strong> - lernen Sie, wie man die ersten N Werte überspringt.</li><li><strong><a href="./take">take</a></strong> - lerne, wie man die ersten N Werte erhält.</li><li><strong><a href="./../utility/takeUntil">takeUntil</a></strong> - lerne, wie man Werte nimmt, bis ein anderes Observable feuert</li><li><strong><a href="./filter">filter</a></strong> - lernen Sie, wie man auf der Grundlage von Bedingungen filtert</li><li><strong><a href="./practical-use-cases">filtering-operator-practical-use-cases</a></strong> - lernen Sie echte Anwendungsfälle</li></ul>`,45)])])}const E=n(p,[["render",t]]);export{g as __pageData,E as default};
