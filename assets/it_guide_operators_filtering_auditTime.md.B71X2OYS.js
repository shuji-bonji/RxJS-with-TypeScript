import{_ as i,o as a,c as n,a2 as p}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse(`{"title":"auditTime - ultimo valore emesso dopo l'ora specificata","description":"auditTime è un operatore di filtraggio di RxJS che attende un tempo specificato per l'emissione di un valore e fornisce l'ultimo valore entro tale periodo. Viene utilizzato soprattutto quando si desidera campionare periodicamente l'ultimo stato di eventi ad alta frequenza, come il tracciamento della posizione dello scroll, il ridimensionamento della finestra, il movimento del mouse, ecc.","frontmatter":{"description":"auditTime è un operatore di filtraggio di RxJS che attende un tempo specificato per l'emissione di un valore e fornisce l'ultimo valore entro tale periodo. Viene utilizzato soprattutto quando si desidera campionare periodicamente l'ultimo stato di eventi ad alta frequenza, come il tracciamento della posizione dello scroll, il ridimensionamento della finestra, il movimento del mouse, ecc."},"headers":[],"relativePath":"it/guide/operators/filtering/auditTime.md","filePath":"it/guide/operators/filtering/auditTime.md","lastUpdated":1779057779000}`),e={name:"it/guide/operators/filtering/auditTime.md"};function l(t,s,h,o,k,r){return a(),n("div",null,[...s[0]||(s[0]=[p(`<h1 id="audittime-ultimo-valore-emesso-dopo-l-ora-specificata" tabindex="-1">auditTime - ultimo valore emesso dopo l&#39;ora specificata <a class="header-anchor" href="#audittime-ultimo-valore-emesso-dopo-l-ora-specificata" aria-label="Permalink to &quot;auditTime - ultimo valore emesso dopo l&#39;ora specificata&quot;">​</a></h1><p>L&#39;operatore <code>auditTime</code> attende un <strong>tempo specificato</strong> dopo l&#39;emissione di un valore e produce l&#39;<strong>ultimo valore</strong> entro tale periodo. Quindi attende l&#39;arrivo del valore successivo.</p><h2 id="🔰-sintassi-e-utilizzo-di-base" tabindex="-1">🔰 Sintassi e utilizzo di base <a class="header-anchor" href="#🔰-sintassi-e-utilizzo-di-base" aria-label="Permalink to &quot;🔰 Sintassi e utilizzo di base&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Fare clic.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span></code></pre></div><p><strong>Flusso delle operazioni</strong>:.</p><ol><li>si verifica il primo clic</li><li>attende 1 secondo (i clic durante questo tempo vengono registrati ma non emessi)</li><li>emette l&#39;ultimo clic dopo 1 secondo Attendere il clic successivo</li></ol><p><a href="https://rxjs.dev/api/operators/auditTime" target="_blank" rel="noreferrer">🌐 Documentazione ufficiale RxJS - auditTime</a></p><h2 id="🆚-contrasto-con-throttletime" tabindex="-1">🆚 Contrasto con throttleTime <a class="header-anchor" href="#🆚-contrasto-con-throttletime" aria-label="Permalink to &quot;🆚 Contrasto con throttleTime&quot;">​</a></h2><p>throttleTime e auditTime sono simili, ma differiscono per i valori che producono.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { throttleTime, auditTime, take } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">300</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">take</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// throttleTime: Emissione del primo valore</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  throttleTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uscita.: 0, 4, 8(primo valore di ogni periodo)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// auditTime: Uscita dell&#39;ultimo valore</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uscita.: 3, 6, 9(ultimo valore di ciascun periodo)</span></span></code></pre></div><p><strong>Confronto tra linee temporali</strong>:.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>Fonte:     0--1--2--3--4--5--6--7--8--9--|</span></span>
<span class="line"><span>            |        |        |</span></span>
<span class="line"><span>throttle:   0--------4--------8------------|</span></span>
<span class="line"><span>            (Primo)   (Primo)   (Primo)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>audit:      -------3--------6--------9----|</span></span>
<span class="line"><span>                  (Ultimo)   (Ultimo)   (Ultimo)</span></span></code></pre></div><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Fare clic.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`\`\`ts</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">import { interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">import { throttleTime, auditTime, take } from &#39;rxjs&#39;;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// throttleTime: PrimoのvaloreをOutput</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">source$.pipe(</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">  throttleTime(1000)</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">).subscribe(console.log);</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// Output: 0, 4, 8（各periodoのPrimoのvalore）</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// auditTime: UltimoのvaloreをOutput</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">source$.pipe(</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">  auditTime(1000)</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">).subscribe(console.log);</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// Output: 3, 6, 9（各periodoのUltimoのvalore）</span></span></code></pre></div><h2 id="💡-modello-di-utilizzo-tipico" tabindex="-1">💡 Modello di utilizzo tipico <a class="header-anchor" href="#💡-modello-di-utilizzo-tipico" aria-label="Permalink to &quot;💡 Modello di utilizzo tipico&quot;">​</a></h2><ol><li><strong>Ottimizzare il ridimensionamento della finestra</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;resize&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">200</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">) </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 200msOttenere l&#39;ultima dimensione nell&#39;intervallo</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Dimensione della finestra: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerWidth</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}x\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerHeight</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span></code></pre></div><ol start="2"><li><p><strong>Tracciamento della posizione di scorrimento</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;scroll&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">100</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  map</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ({</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollY: window.scrollY,</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollX: window.scrollX</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">position</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Posizione di scorrimento: Y=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollY</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}, X=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollX</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li><li><p><strong>Movimento di trascinamento fluido</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map, takeUntil, switchMap } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Creare elementi trascinabili</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> box</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;div&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.width </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.height </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.backgroundColor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;#3498db&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.position </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;absolute&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.cursor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;move&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.left </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.top </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Trascinamento&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Implementare le operazioni di trascinamento</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">mouseDown$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  switchMap</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">startEvent</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startX</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientX </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetLeft;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startY</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientY </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetTop;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    return</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> mouseMove$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">      auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">16</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">), </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Approssimazione.60FPS(vedere anche16ms) per aggiornare la posizione</span></span>
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
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li></ol><h2 id="🧠-esempio-pratico-di-codice-tracciamento-del-mouse" tabindex="-1">🧠 Esempio pratico di codice (tracciamento del mouse) <a class="header-anchor" href="#🧠-esempio-pratico-di-codice-tracciamento-del-mouse" aria-label="Permalink to &quot;🧠 Esempio pratico di codice (tracciamento del mouse)&quot;">​</a></h2><p>Questo esempio traccia i movimenti del mouse e visualizza l&#39;ultima posizione a intervalli regolari.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Creare gli elementi dell&#39;interfaccia utente</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>container.style.height = &#39;300px&#39;;</span></span>
<span class="line"><span>container.style.border = &#39;2px solid #3498db&#39;;</span></span>
<span class="line"><span>container.style.padding = &#39;20px&#39;;</span></span>
<span class="line"><span>container.style.position = &#39;relativo&#39;;</span></span>
<span class="line"><span>container.textContent = &#39;Muovi il mouse in quest&#39;area&#39;;</span></span>
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
<span class="line"><span>dot.style.position = &#39;absolute&#39;;</span></span>
<span class="line"><span>dot.style.display = &#39;none&#39;;</span></span>
<span class="line"><span>container.appendChild(dot);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Evento di spostamento del mouse</span></span>
<span class="line"><span>fromEvent&lt;MouseEvent&gt;(container, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>  map(evento =&gt; {</span></span>
<span class="line"><span>    const rect = container.getBoundingClientRect();</span></span>
<span class="line"><span>    return {</span></span>
<span class="line"><span>      x: event.clientX - rect.left,.</span></span>
<span class="line"><span>      y: event.clientY - rect.top</span></span>
<span class="line"><span>    };</span></span>
<span class="line"><span>  }),</span></span>
<span class="line"><span>  auditTime(100) // Ottiene l&#39;ultima posizione ogni 100ms</span></span>
<span class="line"><span>).subscribe(posizione =&gt; {</span></span>
<span class="line"><span>  positionDisplay.textContent = \`Ultima posizione (ogni 100 ms): X=\${posizione.x.toFixed(0)}, Y=\${posizione.y.toFixed(0)}\`;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Sposta il punto nell&#39;ultima posizione</span></span>
<span class="line"><span>  dot.style.left = \`\${position.x - 5}px\`;</span></span>
<span class="line"><span>  dot.style.top = \`\${position.y - 5}px\`;</span></span>
<span class="line"><span>  dot.style.display = &#39;block&#39;;</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Questo codice recupera e visualizza l&#39;ultima posizione solo ogni volta che il mouse viene spostato, anche se il mouse viene spostato frequentemente,100msIl codice recupera e visualizza l&#39;ultima posizione solo per ogni movimento del mouse.</p><h2 id="🎯-debouncetime-le-differenze-tra" tabindex="-1">🎯 debounceTime Le differenze tra <a class="header-anchor" href="#🎯-debouncetime-le-differenze-tra" aria-label="Permalink to &quot;🎯 debounceTime Le differenze tra&quot;">​</a></h2><p><code>auditTime</code> e <code>debounceTime</code> è che<strong>entrambi producono l&#39;ultimo valore</strong>ma il codice<strong>La tempistica è completamente diversa</strong>l&#39;ultimo valore viene emesso.</p><h3 id="la-differenza-decisiva" tabindex="-1">La differenza decisiva <a class="header-anchor" href="#la-differenza-decisiva" aria-label="Permalink to &quot;La differenza decisiva&quot;">​</a></h3><table tabindex="0"><thead><tr><th>Operatore</th><th>operazione</th><th>utilizzo del sistema in modi diversi</th></tr></thead><tbody><tr><td><code>auditTime(ms)</code></td><td>Quando arriva un valore<strong>msSempre in uscita dopo</strong>(anche se l&#39;ingresso continua)</td><td>Campionamento periodico</td></tr><tr><td><code>debounceTime(ms)</code></td><td><strong>Dopo che l&#39;ingresso si è fermato</strong>msUscita dopo</td><td>Attendere il completamento dell&#39;ingresso</td></tr></tbody></table><h3 id="esempi-specifici-differenze-nell-input-di-ricerca" tabindex="-1">Esempi specifici：Differenze nell&#39;input di ricerca <a class="header-anchor" href="#esempi-specifici-differenze-nell-input-di-ricerca" aria-label="Permalink to &quot;Esempi specifici：Differenze nell&#39;input di ricerca&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, debounceTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.placeholder = &#39;Input parola di ricerca&#39;;</span></span>
<span class="line"><span>document.body.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// AuditTime: Eseguire la ricerca ogni 300 ms anche durante l&#39;inserimento di input</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300)</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;auditTime → Ricerca:&#39;, input.value);</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// debounceTime: attendere 300 ms dopo che l&#39;input si è fermato, quindi eseguire la ricerca</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300)</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;debounceTime → Ricerca:&#39;, input.value);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h3 id="differenze-viste-nella-timeline" tabindex="-1">Differenze viste nella timeline <a class="header-anchor" href="#differenze-viste-nella-timeline" aria-label="Permalink to &quot;Differenze viste nella timeline&quot;">​</a></h3><p>Differenza vista quando un utente fa clic su &quot;ab&#39;→&#39;abc&#39;→&#39;abcd&#39; durante la digitazione veloce:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>Evento di input: a--b--c--d------------|</span></span>
<span class="line"><span>              ↓</span></span>
<span class="line"><span>auditTime: ------c-----d----------|</span></span>
<span class="line"><span>            (dopo 300 ms) (dopo 300 ms)</span></span>
<span class="line"><span>            → Cerca &#39;abc&#39;, cerca &#39;abcd&#39; (2 volte in totale)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>debounceTime: --------------------d-|</span></span>
<span class="line"><span>                              (300 ms dopo l&#39;arresto)</span></span>
<span class="line"><span>            → Cerca &quot;abcd&quot; (solo una volta in totale)</span></span></code></pre></div><p><strong>Facile da ricordare</strong>:</p><ul><li><strong><code>auditTime</code></strong>: &#39;Controllato regolarmente (audit)&quot;→ &#39;Controllate sempre a intervalli regolari&#39;</li><li><strong><code>debounceTime</code></strong>: &#39;Aspettare che si sia calmato (...)&#39;.debounceAspettare che sia tranquillo.→ &#39;Aspettare finché non è tranquillo&#39;</li></ul><h3 id="uso-pratico" tabindex="-1">Uso pratico <a class="header-anchor" href="#uso-pratico" aria-label="Permalink to &quot;Uso pratico&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>// ✅ AuditTime se appropriato</span></span>
<span class="line"><span>// - Tracciamento della posizione di scorrimento (vogliamo ottenerla periodicamente, anche se stiamo scorrendo tutto il tempo)</span></span>
<span class="line"><span>fromEvent(window, &#39;scroll&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(100) // ottiene l&#39;ultima posizione ogni 100 ms</span></span>
<span class="line"><span>).subscribe(/* ... */);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ se debounceTime è appropriato.</span></span>
<span class="line"><span>// - casella di ricerca (vogliamo effettuare la ricerca al termine dell&#39;input)</span></span>
<span class="line"><span>fromEvent(searchInput, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300) // aspetta 300 ms dopo l&#39;interruzione dell&#39;input</span></span>
<span class="line"><span>).subscribe(/* ... */);</span></span></code></pre></div><h2 id="📋-utilizzo-sicuro-per-i-tipi" tabindex="-1">📋 Utilizzo sicuro per i tipi <a class="header-anchor" href="#📋-utilizzo-sicuro-per-i-tipi" aria-label="Permalink to &quot;📋 Utilizzo sicuro per i tipi&quot;">​</a></h2><p>TypeScript Questo è un esempio di implementazione type-safe che fa uso dei generici in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaccia MousePosition {</span></span>
<span class="line"><span>  x: numero;</span></span>
<span class="line"><span>  y: numero</span></span>
<span class="line"><span>  timestamp: numero; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function trackMousePosition(</span></span>
<span class="line"><span>  elemento: HTMLElement,.</span></span>
<span class="line"><span>  intervalli: numero</span></span>
<span class="line"><span>): Observable {</span></span>
<span class="line"><span>  return fromEvent&lt;MouseEvent&gt;(element, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>    auditTime(intervalli),.</span></span>
<span class="line"><span>    map(event =&gt; ({ {</span></span>
<span class="line"><span>      x: event.clientX, event.</span></span>
<span class="line"><span>      y: event.clientY,.</span></span>
<span class="line"><span>      timestamp: Date.now())</span></span>
<span class="line"><span>    } come MousePosition))</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Esempio di utilizzo</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>canvas.style.width = &#39;400px&#39;;</span></span>
<span class="line"><span>canvas.style.height = &#39;300px&#39;;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px solid black&#39;;</span></span>
<span class="line"><span>document.body.appendChild(canvas);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>trackMousePosition(canvas, 200).subscribe(position =&gt; {</span></span>
<span class="line"><span>  console.log(\`Posizione: (\${posizione.x}, \${posizione.y}) a \${posizione.timestamp}\`);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h2 id="🔄-audittime-e-throttletime-combinazione-di" tabindex="-1">🔄 auditTime e throttleTime Combinazione di <a class="header-anchor" href="#🔄-audittime-e-throttletime-combinazione-di" aria-label="Permalink to &quot;🔄 auditTime e throttleTime Combinazione di&quot;">​</a></h2><p>In alcuni scenari, entrambi possono essere combinati.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importare { throttleTime, auditTime, take } da &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(100).pipe(take(50));.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ordine di throttleTime → auditTime</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  throttleTime(1000), // passa il primo valore ogni secondo</span></span>
<span class="line"><span>  auditTime(500) // quindi attende 500 ms e invia l&#39;ultimo valore</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Fare clic.！&#39;));</span></span>
<span class="line"><span>---</span></span>
<span class="line"><span>description: auditTimeは値が発行されたら指定時間待機し、その期間内の最後の値を出力するRxJSフィルタリングオペレーターです。スクロール位置の追跡、ウィンドウリサイズ、マウス移動などの高頻度イベントで最新の状態を定期的にサンプリングしたい場合に最適です。throttleTimeやdebounceTimeとの違いを理解して適切に使い分けることが重要です。</span></span>
<span class="line"><span>---</span></span>
<span class="line"><span></span></span>
<span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Creare un campo di input per la ricerca</span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);.</span></span>
<span class="line"><span>input.type = &#39;text&#39;;</span></span>
<span class="line"><span>input.placeholder = &#39;Cerca...&#39; ;</span></span>
<span class="line"><span>document.body.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Cattivo esempio: usare auditTime per l&#39;input di ricerca</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300) // la ricerca viene eseguita ogni 300 ms durante l&#39;input</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Ricerca eseguita&#39;);</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Fare clic.！&#39;));</span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;クリック！&#39;));</span></span></code></pre></div><p>ts. import { fromEvent } from &#39;rxjs&#39;; import { debounceTime } from &#39;rxjs&#39;;</p><p>// Creare un campo di input per la ricerca const input = document.createElement(&#39;input&#39;);. input.type = &#39;text&#39;; input.placeholder = &#39;Cerca...&#39; ; document.body.appendChild(input);</p><p>// ✅ Buon esempio: usare debounceTime per l&#39;input di ricerca fromEvent(input, &#39;input&#39;).pipe( debounceTime(300) // Attendere 300 ms dopo l&#39;arresto dell&#39;input prima di eseguire la ricerca ).subscribe(() =&gt; { console.log(&#39;Ricerca eseguita&#39;, input.value); });</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>## 🎓 Riepilogo</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Quando si dovrebbe usare auditTime.</span></span>
<span class="line"><span>- ✅ Quando sono richiesti valori aggiornati a intervalli regolari</span></span>
<span class="line"><span>- ✅ Eventi ad alta frequenza come scorrimento, ridimensionamento, movimento del mouse</span></span>
<span class="line"><span>- ✅ Quando è richiesto un campionamento periodico</span></span>
<span class="line"><span>- ✅ Quando si desidera riflettere lo stato più recente.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Quando si deve usare throttleTime.</span></span>
<span class="line"><span>- ✅ Quando è richiesta una risposta immediata</span></span>
<span class="line"><span>- ✅ Se si vuole iniziare l&#39;elaborazione con il primo valore</span></span>
<span class="line"><span>- ✅ Prevenzione dello schiacciamento dei tasti</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Quando utilizzare debounceTime.</span></span>
<span class="line"><span>- ✅ Se si desidera attendere il completamento dell&#39;input</span></span>
<span class="line"><span>- ✅ Ricerca, completamento automatico</span></span>
<span class="line"><span>- ✅ Aspettare che l&#39;utente smetta di digitare.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Note.</span></span>
<span class="line"><span>- ⚠️ L&#39;auditTime produce solo l&#39;ultimo valore del periodo (i valori intermedi vengono scartati).</span></span>
<span class="line"><span>- ⚠️ Non è molto efficace se impostato per intervalli brevi.</span></span>
<span class="line"><span>- ⚠️ throttleTime\` o debounceTime\` possono essere più appropriati a seconda dell&#39;applicazione.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>## 🚀 Prossimi passi.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>- **[throttleTime](. /throttleTime)** - imparare a passare il primo valore.</span></span>
<span class="line"><span>- **[debounceTime](. /debounceTime)** - imparare a emettere valori dopo l&#39;interruzione dell&#39;input.</span></span>
<span class="line"><span>- **[filter](. /filter)** - impara a filtrare in base a delle condizioni.</span></span>
<span class="line"><span>- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - per imparare a utilizzare casi d&#39;uso reali.</span></span></code></pre></div>`,44)])])}const E=i(e,[["render",l]]);export{c as __pageData,E as default};
