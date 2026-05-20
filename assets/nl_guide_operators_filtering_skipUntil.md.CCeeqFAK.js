import{_ as a,o as n,c as i,a2 as e}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse('{"title":"skipUntil - overslaan naar ontsteking","description":"De skipUntil operator slaat alle waarden van de oorspronkelijke Observable over totdat een andere Observable een waarde afgeeft, waarna de waarde als normaal wordt uitgevoerd. Dit is handig voor op tijd gebaseerde uitgestelde starts of nadat een specifieke gebeurtenis heeft plaatsgevonden.","frontmatter":{"description":"De skipUntil operator slaat alle waarden van de oorspronkelijke Observable over totdat een andere Observable een waarde afgeeft, waarna de waarde als normaal wordt uitgevoerd. Dit is handig voor op tijd gebaseerde uitgestelde starts of nadat een specifieke gebeurtenis heeft plaatsgevonden."},"headers":[],"relativePath":"nl/guide/operators/filtering/skipUntil.md","filePath":"nl/guide/operators/filtering/skipUntil.md","lastUpdated":1779269732000}'),p={name:"nl/guide/operators/filtering/skipUntil.md"};function l(t,s,k,h,r,d){return n(),i("div",null,[...s[0]||(s[0]=[e(`<h1 id="skipuntil-overslaan-naar-ontsteking" tabindex="-1">skipUntil - overslaan naar ontsteking <a class="header-anchor" href="#skipuntil-overslaan-naar-ontsteking" aria-label="Permalink to &quot;skipUntil - overslaan naar ontsteking&quot;">​</a></h1><p>De operator <code> skipUntil</code> slaat alle waarden van de oorspronkelijke Observable** over totdat de eerste waarde wordt afgegeven door de gespecificeerde Observable (kennisgevingstrigger). Na het tijdstip waarop de kennisgevingstrigger wordt afgegeven, worden de waarden zoals gebruikelijk uitgevoerd.</p><h2 id="🔰-basissyntaxis-en-gebruik" tabindex="-1">🔰 Basissyntaxis en gebruik <a class="header-anchor" href="#🔰-basissyntaxis-en-gebruik" aria-label="Permalink to &quot;🔰 Basissyntaxis en gebruik&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Waarde elke seconde uitgeven</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Waarde na elke seconde uitgeven</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// (eerste2tweede waarde 0, 1, 2, 3 worden overgeslagen)</span></span></code></pre></div><p><strong>Bewerkingsstroom</strong>:.</p><ol><li><code>bron$</code> geeft 0, 1, 2, 3 → alles overslaan</li><li>2 seconden later geeft <code>melder$</code> een waarde uit</li><li>volgende <code>bron$</code> waarden (4, 5, 6, ...) worden zoals gewoonlijk uitgevoerd.</li></ol><p><a href="https://rxjs.dev/api/operators/skipUntil" target="_blank" rel="noreferrer">🌐 Officiële RxJS documentatie - <code> skipUntil</code></a></p><h2 id="🆚-contrast-met-takeuntil" tabindex="-1">🆚 Contrast met takeUntil <a class="header-anchor" href="#🆚-contrast-met-takeuntil" aria-label="Permalink to &quot;🆚 Contrast met takeUntil&quot;">​</a></h2><p><code>skipUntil</code> en <code>takeUntil</code> hebben contrasterend gedrag.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil, takeUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Waarde elke seconde uitgeven</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Waarde na elke seconde uitgeven</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// takeUntil: Waarde ophalen tot melding</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  takeUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang: 0, 1, 2, 3(Stopt na2(stopt na 1,5 seconden)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// skipUntil: Waarden overslaan tot melding</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang: 4, 5, 6, 7, ...(Stopt na2(Begint na 1,5 seconden)</span></span></code></pre></div><p>TABEL 10</p><h2 id="💡-typisch-gebruikspatroon" tabindex="-1">💡 Typisch gebruikspatroon <a class="header-anchor" href="#💡-typisch-gebruikspatroon" aria-label="Permalink to &quot;💡 Typisch gebruikspatroon&quot;">​</a></h2><ol><li><strong>Begin met het verwerken van gegevens na authenticatie van de gebruiker</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, Subject } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> authenticated$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> new</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Subject</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">void</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;();</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> dataStream$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Gegevens overslaan tot authenticatie is voltooid</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   dataStream$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(authenticated$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">data</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Gegevensverwerking: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">data</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // 3Authenticatie voltooid na 2 seconden</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   setTimeout</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Authenticatie voltooid！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     authenticated$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">next</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">();</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   }, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // 3(Begint na seconde) &quot;Gegevensverwerking: 3Gegevensverwerking: 4&#39;, &#39;Gegevensverwerking...en uitvoer</span></span></code></pre></div><ol start="2"><li><p><strong>Gebeurtenisverwerking start na voltooiing van initieel laden</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent, BehaviorSubject } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { filter, skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> appReady$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> new</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> BehaviorSubject</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">boolean</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">false</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> button</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;button&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">button.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Klikken.&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> clicks$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Negeer klikken totdat de app klaar is</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">clicks$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(appReady$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">filter</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">ready</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ready)))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Klik verwerkt&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2App klaar in seconden</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">setTimeout</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;App is klaar&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  appReady$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">next</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">true</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span></code></pre></div></li><li><p><strong>Op timer gebaseerde vertraging gestart</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil, scan } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> button</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;button&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">button.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Tellen&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> clicks$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startTime$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 3Seconden later</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 3Kliks worden pas geteld nadat seconden zijn verstreken</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">clicks$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(startTime$),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  scan</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">count</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> count </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">+</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">count</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Tellen: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">count</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;3Tellen begint na seconden...&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span></code></pre></div></li></ol><h2 id="🧠-praktisch-codevoorbeeld-aftellen-van-spel" tabindex="-1">🧠 Praktisch codevoorbeeld (aftellen van spel) <a class="header-anchor" href="#🧠-praktisch-codevoorbeeld-aftellen-van-spel" aria-label="Permalink to &quot;🧠 Praktisch codevoorbeeld (aftellen van spel)&quot;">​</a></h2><p>Dit is een voorbeeld van het negeren van klikken tijdens het aftellen voordat het spel begint en het inschakelen van klikken nadat het aftellen is afgelopen.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent, timer, interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, take, scan } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// UI-elementen maken</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const countdown = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>countdown.style.fontSize = &#39;24px&#39;;</span></span>
<span class="line"><span>countdown.style.marginBottom = &#39;10px&#39;;</span></span>
<span class="line"><span>countdown.textContent = &#39;Aftellen bezig...&#39;. ;</span></span>
<span class="line"><span>container.appendChild(countdown);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const button = document.createElement(&#39;button&#39;);</span></span>
<span class="line"><span>button.textContent = &#39;Klik! ;</span></span>
<span class="line"><span>button.disabled = true;</span></span>
<span class="line"><span>container.appendChild(button);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const scoreDisplay = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>scoreDisplay.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>scoreDisplay.textContent = &#39;score: 0&#39;;</span></span>
<span class="line"><span>container.appendChild(scoreDisplay);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Aftellen (3 seconden)</span></span>
<span class="line"><span>const countdownTimer$ = interval(1000).pipe(take(3));</span></span>
<span class="line"><span>countdownTimer$.subscribe({</span></span>
<span class="line"><span>  next: (n) =&gt; {</span></span>
<span class="line"><span>    countdown.textContent = \`\${3 - n} seconden tot start... \`;</span></span>
<span class="line"><span>  },.</span></span>
<span class="line"><span>  complete: () =&gt; {</span></span>
<span class="line"><span>    countdown.textContent = \`Het spel begint! ;</span></span>
<span class="line"><span>    knop.uitgeschakeld = onwaar;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Spelstartmelding</span></span>
<span class="line"><span>const gameStart$ = timer(3000);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Klikgebeurtenis (slaat over naar het begin van het spel)</span></span>
<span class="line"><span>const clicks$ = fromEvent(button, &#39;click&#39;);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>clicks$.pipe(</span></span>
<span class="line"><span>  skipUntil(gameStart$),.</span></span>
<span class="line"><span>  scan(score =&gt; score + 10, 0)</span></span>
<span class="line"><span>).subscribe(score =&gt; {</span></span>
<span class="line"><span>  scoreDisplay.textContent = \`score: \${score}\`;</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>In deze code is het aftellen3seconden, worden klikken tijdens het aftellen genegeerd en worden alleen klikken na het aftellen weergegeven in de score.</p><h2 id="🎯-skip-het-verschil-tussen-skipuntil-verschil-tussen" tabindex="-1">🎯 skip Het verschil tussen skipUntil Verschil tussen <a class="header-anchor" href="#🎯-skip-het-verschil-tussen-skipuntil-verschil-tussen" aria-label="Permalink to &quot;🎯 skip Het verschil tussen skipUntil Verschil tussen&quot;">​</a></h2><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skip, skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// skip: sla de eerste N per getal over</span></span>
<span class="line"><span>bron$.pipe(</span></span>
<span class="line"><span>  skip(3)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// uitvoer: 3, 4, 5, 6, ...</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// skipUntil: overslaan totdat een andere Observable afgaat</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(timer(1500))</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// uitvoer: 3, 4, 5, 6, ... (hetzelfde resultaat, maar andere controlemethode)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Voorwaarden voor overslaan</th><th>Gebruik</th></tr></thead><tbody><tr><td><code>skip(n)</code></td><td>EerstenSla een aantal stukken over</td><td>Een vast aantal overslaan</td></tr><tr><td><code>skipWhile(predicate)</code></td><td>Overslaan terwijl aan voorwaarden is voldaan</td><td>Overslaan op basis van voorwaarden</td></tr><tr><td><code>skipUntil(notifier$)</code></td><td>Overslaan tot een anderObservableOverslaan tot een</td><td>Gebeurtenis/Overslaan op basis van tijd</td></tr></tbody></table><h2 id="📋-type-veilig-gebruik" tabindex="-1">📋 Type-veilig gebruik <a class="header-anchor" href="#📋-type-veilig-gebruik" aria-label="Permalink to &quot;📋 Type-veilig gebruik&quot;">​</a></h2><p>TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, Subject, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface GameState {</span></span>
<span class="line"><span>  status: &#39;waiting&#39; | &#39;ready&#39; | &#39;playing&#39; | &#39;finished&#39;;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface ClickEvent {</span></span>
<span class="line"><span>  timestamp: getal; }</span></span>
<span class="line"><span>  x: getal;</span></span>
<span class="line"><span>  y: getal;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>klasse Spel {</span></span>
<span class="line"><span>  private gameReady$ = new Subject();</span></span>
<span class="line"><span>  private state: GameState = { status: &#39;waiting&#39; };.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  startGame(element: HTMLElement): Observable {</span></span>
<span class="line"><span>    const clicks$ = fromEvent\\&lt;MouseEvent&gt;(element, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>      map(event =&gt; ({</span></span>
<span class="line"><span>        timestamp: Date.now(),.</span></span>
<span class="line"><span>        x: event.clientX, event.</span></span>
<span class="line"><span>        y: event.clientY</span></span>
<span class="line"><span>      } as ClickEvent)),.</span></span>
<span class="line"><span>      skipUntil(this.gameReady$)</span></span>
<span class="line"><span>    );</span></span>
<span class="line"><span></span></span>
<span class="line"><span>    // Melding van gereedheid</span></span>
<span class="line"><span>    setTimeout() =&gt; {</span></span>
<span class="line"><span>      this.state = {status: &#39;klaar&#39; };</span></span>
<span class="line"><span>      this.gameReady$.next();</span></span>
<span class="line"><span>      console.log(&#39;Spel klaar!&#39;) ;</span></span>
<span class="line"><span>    }, 2000);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>    return clicks$;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Gebruiksvoorbeeld</span></span>
<span class="line"><span>spel = nieuw spel();</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>canvas.style.width = &#39;300px&#39;;</span></span>
<span class="line"><span>canvas.style.height = &#39;200px&#39;;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px solid black&#39;;</span></span>
<span class="line"><span>canvas.textContent = &#39;Klik hier&#39;;</span></span>
<span class="line"><span>document.body.appendChild(canvas);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>game.startGame(canvas).subscribe(click =&gt; {</span></span>
<span class="line"><span>  console.log(\`Klik positie: (\${klik.x}, \${klik.y})\`);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h2 id="🔄-skipuntil-het-verschil-tussen-takeuntil-combinatie-van" tabindex="-1">🔄 skipUntil Het verschil tussen takeUntil Combinatie van <a class="header-anchor" href="#🔄-skipuntil-het-verschil-tussen-takeuntil-combinatie-van" aria-label="Permalink to &quot;🔄 skipUntil Het verschil tussen takeUntil Combinatie van&quot;">​</a></h2><p>Combineer beide als je alleen waarden wilt krijgen voor een specifieke tijdsperiode.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, takeUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500);</span></span>
<span class="line"><span>const start$ = timer(2000); // start na 2 seconden</span></span>
<span class="line"><span>const stop$ = timer(5000); // stop na 5 seconden</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(start$), // sla over tot na 2 seconden</span></span>
<span class="line"><span>  takeUntil(stop$); // stop na 5 seconden</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uitvoer: 4, 5, 6, 7, 8, 9, voltooien.</span></span>
<span class="line"><span>// (alleen waarden tussen 2 en 5 seconden worden opgehaald)</span></span></code></pre></div><p><strong>Tijdlijnen</strong>:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>0s 1s 2s 3s 4s 5s</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500); // 0.5Waarde elke seconde uitgeven</span></span>
<span class="line"><span>const notifier$ = timer(2000); // 2Waarde na elke seconde uitgeven</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(notifier$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// Uitgang: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span>// (eerste2tweede waarde 0, 1, 2, 3 worden overgeslagen)</span></span></code></pre></div><p>0 1 2 3 4 5 6 7 8 9 10 ↑ omhoog omhoog omhoog omhoog SKIP start TAKE einde (vanaf 4) (tot 9)</p><h2 id="⚠️-een-veelgemaakte-fout" tabindex="-1">⚠️ Een veelgemaakte fout <a class="header-anchor" href="#⚠️-een-veelgemaakte-fout" aria-label="Permalink to &quot;⚠️ Een veelgemaakte fout&quot;">​</a></h2><div class="important custom-block github-alert"><p class="custom-block-title">IMPORTANT</p><p><code>skipUntil</code> zijn meldingen Observable van de<strong>Alleen het eerste afvuren</strong>is geldig.2De tweede en volgende ontstekingen worden genegeerd.</p></div><h3 id="vals-meldingobservablewordt-meer-dan-een-keer-geactiveerd" tabindex="-1">Vals: MeldingObservablewordt meer dan één keer geactiveerd. <a class="header-anchor" href="#vals-meldingobservablewordt-meer-dan-een-keer-geactiveerd" aria-label="Permalink to &quot;Vals: MeldingObservablewordt meer dan één keer geactiveerd.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { interval, Subject } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const bron$ = interval(500);</span></span>
<span class="line"><span>const melder$ = nieuw Subject();</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(kennisgever$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Slecht voorbeeld: roep next meerdere keren aan, maar alleen de eerste heeft effect</span></span>
<span class="line"><span>setTimeout() =&gt; notifier$.next(), 1000);</span></span>
<span class="line"><span>setTimeout() =&gt; notifier$.next(), 2000); // dit is zinloos</span></span></code></pre></div><h3 id="correct-begrijp-dat-alleen-de-eerste-afvuring-geldig-is" tabindex="-1">Correct.: Begrijp dat alleen de eerste afvuring geldig is. <a class="header-anchor" href="#correct-begrijp-dat-alleen-de-eerste-afvuring-geldig-is" aria-label="Permalink to &quot;Correct.: Begrijp dat alleen de eerste afvuring geldig is.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, Subject } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const bron$ = interval(500);</span></span>
<span class="line"><span>const melder$ = nieuw Subject();</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(kennisgever$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Goed voorbeeld: roep next maar één keer aan</span></span>
<span class="line"><span>setTimeout() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Einde overslaan&#39;);</span></span>
<span class="line"><span>  notifier$.next();</span></span>
<span class="line"><span>  notifier$.complete(); // best practice om te completeren.</span></span>
<span class="line"><span>}, 1000);</span></span></code></pre></div><h2 id="samenvatting" tabindex="-1">Samenvatting <a class="header-anchor" href="#samenvatting" aria-label="Permalink to &quot;Samenvatting&quot;">​</a></h2><h3 id="wanneer-skipuntil-moet-worden-gebruikt" tabindex="-1">Wanneer skipUntil moet worden gebruikt. <a class="header-anchor" href="#wanneer-skipuntil-moet-worden-gebruikt" aria-label="Permalink to &quot;Wanneer skipUntil moet worden gebruikt.&quot;">​</a></h3><ul><li>✅ Als u de verwerking wilt starten nadat een specifieke gebeurtenis heeft plaatsgevonden</li><li>✅ Als u gebruikersbewerkingen wilt inschakelen nadat de initialisatie is voltooid</li><li>✅ Als u een op tijd gebaseerde uitgestelde start nodig hebt</li><li>✅ Als u de gegevensverwerking wilt starten nadat de authenticatie is voltooid</li></ul><h3 id="in-combinatie-met-takeuntil" tabindex="-1">In combinatie met takeUntil. <a class="header-anchor" href="#in-combinatie-met-takeuntil" aria-label="Permalink to &quot;In combinatie met takeUntil.&quot;">​</a></h3><ul><li>✅ Als u alleen waarden voor een bepaalde periode wilt krijgen (skipUntil + takeUntil)</li></ul><h3 id="opmerkingen" tabindex="-1">Opmerkingen. <a class="header-anchor" href="#opmerkingen" aria-label="Permalink to &quot;Opmerkingen.&quot;">​</a></h3><ul><li>⚠️ Alleen het eerste afvuren van de Observable is geldig.</li><li>⚠️ Als de Observable niet afgaat, worden alle waarden nog steeds overgeslagen.</li><li>⚠️ Abonnement wordt gehandhaafd totdat de oorspronkelijke stroom is voltooid</li></ul><h2 id="volgende-stappen" tabindex="-1">Volgende stappen. <a class="header-anchor" href="#volgende-stappen" aria-label="Permalink to &quot;Volgende stappen.&quot;">​</a></h2><ul><li><strong><a href="./skip">skip</a></strong> - leer hoe je de eerste N waarden kunt overslaan.</li><li><strong><a href="./take">take</a></strong> - leren hoe je de eerste N waarden krijgt.</li><li><strong><a href="./../utility/takeUntil">takeUntil</a></strong> - leer hoe u waarden kunt nemen totdat een andere Observable afgaat.</li><li>Filter](./filter)** - leren filteren op basis van voorwaarden</li><li><strong><a href="./practical-use-cases">filtering-operator-praktische-gebruiksgevallen</a></strong> - leer echte use-cases</li></ul>`,46)])])}const g=a(p,[["render",l]]);export{c as __pageData,g as default};
