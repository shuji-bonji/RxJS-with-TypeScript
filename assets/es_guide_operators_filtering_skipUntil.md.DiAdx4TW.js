import{_ as a,o as n,c as i,a2 as p}from"./chunks/framework.5Uf62z_5.js";const c=JSON.parse('{"title":"skipUntil - saltar al encendido","description":"El operador skipUntil omite todos los valores del Observable original hasta que otro Observable emite un valor, tras lo cual el valor se emite de forma normal. Esto es útil para inicios retardados basados en el tiempo o después de que se haya producido un evento específico.","frontmatter":{"description":"El operador skipUntil omite todos los valores del Observable original hasta que otro Observable emite un valor, tras lo cual el valor se emite de forma normal. Esto es útil para inicios retardados basados en el tiempo o después de que se haya producido un evento específico."},"headers":[],"relativePath":"es/guide/operators/filtering/skipUntil.md","filePath":"es/guide/operators/filtering/skipUntil.md","lastUpdated":1779269732000}'),e={name:"es/guide/operators/filtering/skipUntil.md"};function l(t,s,h,k,r,o){return n(),i("div",null,[...s[0]||(s[0]=[p(`<h1 id="skipuntil-saltar-al-encendido" tabindex="-1">skipUntil - saltar al encendido <a class="header-anchor" href="#skipuntil-saltar-al-encendido" aria-label="Permalink to &quot;skipUntil - saltar al encendido&quot;">​</a></h1><p>El operador <code>skipUntil</code> <strong>salta todos los valores del Observable</strong> original hasta que el primer valor es emitido por el Observable especificado (trigger de notificación). Después del momento en que se emite el disparador de notificación, los valores se emiten como de costumbre.</p><h2 id="🔰-sintaxis-basica-y-uso" tabindex="-1">🔰 Sintaxis básica y uso <a class="header-anchor" href="#🔰-sintaxis-basica-y-uso" aria-label="Permalink to &quot;🔰 Sintaxis básica y uso&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Emitir valor cada segundo</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Emitir valor cada segundo</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// (primer2segundo valor 0, 1, 2, 3 se omiten)</span></span></code></pre></div><p><strong>Flujo de operación</strong>:.</p><ol><li><code>source$</code> emite 0, 1, 2, 3 → omitir todo</li><li>2 segundos después <code>notifier$</code> emite un valor</li><li>los siguientes valores de <code>source$</code> (4, 5, 6, ...) se emiten como de costumbre.</li></ol><p><a href="https://rxjs.dev/api/operators/skipUntil" target="_blank" rel="noreferrer">Documentación oficial de RxJS - <code>skipUntil</code></a></p><h2 id="🆚-contraste-con-takeuntil" tabindex="-1">🆚 Contraste con takeUntil <a class="header-anchor" href="#🆚-contraste-con-takeuntil" aria-label="Permalink to &quot;🆚 Contraste con takeUntil&quot;">​</a></h2><p><code>skipUntil</code> y <code>takeUntil</code> tienen comportamientos opuestos.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil, takeUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Emitir valor cada segundo</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Emitir valor cada segundo</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// takeUntil: Recuperar valor hasta que se notifique</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  takeUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida: 0, 1, 2, 3(se detiene después de2(se detiene después de 1,5 segundos)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// skipUntil: Omitir valores hasta que se notifique</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida: 4, 5, 6, 7, ...(se detiene después de2(Se inicia después de 1,5 segundos)</span></span></code></pre></div><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">500</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0.5Emitir valor cada segundo</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> notifier$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Emitir valor cada segundo</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(notifier$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// (primer2segundo valor 0, 1, 2, 3 se omiten)</span></span></code></pre></div><h2 id="💡-patron-tipico-de-utilizacion" tabindex="-1">💡 Patrón típico de utilización <a class="header-anchor" href="#💡-patron-tipico-de-utilizacion" aria-label="Permalink to &quot;💡 Patrón típico de utilización&quot;">​</a></h2><ol><li><strong>Inicio del procesamiento de datos tras la autenticación del usuario</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval, Subject } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> authenticated$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> new</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Subject</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">void</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;();</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> dataStream$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Omitir datos hasta que se complete la autenticación</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   dataStream$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(authenticated$)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">data</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Procesamiento de datos: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">data</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // 3Autenticación completada tras 2 segundos</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   setTimeout</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Autenticación completada！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     authenticated$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">next</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">();</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   }, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // 3(Comienza después de 2 segundos) &quot;Tratamiento de datos: 3Tratamiento de datos: 4&#39;, &quot;Tratamiento de datos...y salida</span></span></code></pre></div><ol start="2"><li><p><strong>El procesamiento de eventos comienza tras la finalización de la carga inicial</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent, BehaviorSubject } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { filter, skipUntil } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> appReady$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> new</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> BehaviorSubject</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&lt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">boolean</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">&gt;(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">false</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> button</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;button&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">button.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Clics.&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> clicks$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ignora los clics hasta que la aplicación esté lista</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">clicks$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(appReady$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">filter</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">ready</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ready)))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Clic procesado&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 2Aplicación lista en segundos</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">setTimeout</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;La aplicación está lista&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  appReady$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">next</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">true</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span></code></pre></div></li><li><p><strong>Retraso basado en temporizador iniciado</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent, timer } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { skipUntil, scan } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> button</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;button&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">button.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Cuenta&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">document.body.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">appendChild</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> clicks$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(button, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startTime$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> timer</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 3Segundos transcurridos</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 3Los clics no se cuentan hasta que transcurren segundos</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">clicks$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  skipUntil</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(startTime$),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  scan</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">count</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> count </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">+</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">count</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Cuenta: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">count</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;3El recuento comienza pasados los segundos...&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span></code></pre></div></li></ol><h2 id="🧠-ejemplo-practico-de-codigo-cuenta-atras-del-juego" tabindex="-1">🧠 Ejemplo práctico de código (cuenta atrás del juego) <a class="header-anchor" href="#🧠-ejemplo-practico-de-codigo-cuenta-atras-del-juego" aria-label="Permalink to &quot;🧠 Ejemplo práctico de código (cuenta atrás del juego)&quot;">​</a></h2><p>Este es un ejemplo de cómo ignorar los clics durante la cuenta atrás antes de que empiece el juego y activar los clics después de que termine la cuenta atrás.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent, timer, interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, take, scan } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Creación de elementos UI</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(contenedor);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const countdown = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>countdown.style.fontSize = &#39;24px&#39;;</span></span>
<span class="line"><span>countdown.style.marginBottom = &#39;10px&#39;;</span></span>
<span class="line"><span>countdown.textContent = &#39;Cuenta atrás en curso...&#39; ;</span></span>
<span class="line"><span>container.appendChild(countdown);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const button = document.createElement(&#39;button&#39;);</span></span>
<span class="line"><span>button.textContent = &#39;¡Haz clic! ;</span></span>
<span class="line"><span>button.disabled = true;</span></span>
<span class="line"><span>container.appendChild(button);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const scoreDisplay = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>scoreDisplay.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>scoreDisplay.textContent = &#39;puntuación: 0&#39;;</span></span>
<span class="line"><span>container.appendChild(scoreDisplay);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Cuenta atrás (3 segundos)</span></span>
<span class="line"><span>const countdownTimer$ = interval(1000).pipe(take(3));</span></span>
<span class="line"><span>countdownTimer$.subscribe({</span></span>
<span class="line"><span>  next: (n) =&gt; {</span></span>
<span class="line"><span>    countdown.textContent = \`\${3 - n} segundos para empezar... \`;</span></span>
<span class="line"><span>  },.</span></span>
<span class="line"><span>  complete: () =&gt; {</span></span>
<span class="line"><span>    countdown.textContent = \`¡Comienza el juego! ;</span></span>
<span class="line"><span>    button.disabled = false;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Notificación de inicio de juego</span></span>
<span class="line"><span>const gameStart$ = timer(3000);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Evento click (salta al inicio del juego)</span></span>
<span class="line"><span>const clicks$ = fromEvent(button, &#39;click&#39;);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>clicks$.pipe(</span></span>
<span class="line"><span>  skipUntil(gameStart$),.</span></span>
<span class="line"><span>  scan(puntuación =&gt; puntuación + 10, 0)</span></span>
<span class="line"><span>).subscribe(puntuación =&gt; {</span></span>
<span class="line"><span>  scoreDisplay.textContent = \`puntuación: \${puntuación}\`;</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>En este código, la cuenta atrás3segundos, los clics se ignoran durante la cuenta atrás, y sólo los clics después de que la cuenta atrás termine se reflejan en la puntuación.</p><h2 id="🎯-skip-la-diferencia-entre-skipuntil-diferencia-entre" tabindex="-1">🎯 skip La diferencia entre skipUntil Diferencia entre <a class="header-anchor" href="#🎯-skip-la-diferencia-entre-skipuntil-diferencia-entre" aria-label="Permalink to &quot;🎯 skip La diferencia entre skipUntil Diferencia entre&quot;">​</a></h2><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skip, skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const fuente$ = interval(500);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// skip: salta el primer N por número</span></span>
<span class="line"><span>fuente$.pipe(</span></span>
<span class="line"><span>  skip(3)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// salida: 3, 4, 5, 6, ...</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// skipUntil: saltar hasta que se dispare otro Observable</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(timer(1500))</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Salida: 3, 4, 5, 6, ... (mismo resultado, pero diferente método de control)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operador</th><th>Saltar condiciones</th><th>Caso de uso</th></tr></thead><tbody><tr><td><code>skip(n)</code></td><td>Primero ...nOmitir un número de piezas</td><td>Omitir un número fijo</td></tr><tr><td><code>skipWhile(predicate)</code></td><td>Omitir mientras se cumplen las condiciones</td><td>Salto basado en condiciones</td></tr><tr><td><code>skipUntil(notifier$)</code></td><td>Saltar hasta otroObservableSaltar hasta un</td><td>Evento/Omisión basada en el tiempo</td></tr></tbody></table><h2 id="📋-uso-seguro" tabindex="-1">📋 Uso seguro <a class="header-anchor" href="#📋-uso-seguro" aria-label="Permalink to &quot;📋 Uso seguro&quot;">​</a></h2><p>TypeScript Este es un ejemplo de una implementación de tipo seguro que hace uso de los genéricos en</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, Subject, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, map } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaz GameState {</span></span>
<span class="line"><span>  status: &#39;esperando&#39; | &#39;listo&#39; | &#39;jugando&#39; | &#39;terminado&#39;;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaz ClickEvent {</span></span>
<span class="line"><span>  timestamp: número; }</span></span>
<span class="line"><span>  x: número</span></span>
<span class="line"><span>  y: número;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>clase Juego {</span></span>
<span class="line"><span>  private gameReady$ = new Subject();</span></span>
<span class="line"><span>  private estado: GameState = { estado: &#39;esperando&#39; };.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  startGame(element: HTMLElement): Observable {</span></span>
<span class="line"><span>    const clicks$ = fromEvent\\&lt;MouseEvent&gt;(element, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>      map(evento =&gt; ({</span></span>
<span class="line"><span>        timestamp: Date.now(),.</span></span>
<span class="line"><span>        x: evento.clienteX, evento.</span></span>
<span class="line"><span>        y: event.clienteY</span></span>
<span class="line"><span>      } as ClickEvent))),.</span></span>
<span class="line"><span>      skipUntil(this.gameReady$)</span></span>
<span class="line"><span>    );</span></span>
<span class="line"><span></span></span>
<span class="line"><span>    // Notificación de preparación</span></span>
<span class="line"><span>    setTimeout(() =&gt; {</span></span>
<span class="line"><span>      this.state = { status: &#39;ready&#39; };</span></span>
<span class="line"><span>      this.gameReady$.next();</span></span>
<span class="line"><span>      console.log(&#39;¡Juego listo!&#39;) ;</span></span>
<span class="line"><span>    }, 2000);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>    return clicks$;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Ejemplo de uso</span></span>
<span class="line"><span>const juego = nuevo Juego();</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>canvas.style.width = &#39;300px&#39;;</span></span>
<span class="line"><span>canvas.style.height = &#39;200px&#39;;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px negro sólido&#39;;</span></span>
<span class="line"><span>canvas.textContent = &#39;Haz clic aquí&#39;;</span></span>
<span class="line"><span>document.body.appendChild(canvas);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>game.startGame(canvas).subscribe(click =&gt; {</span></span>
<span class="line"><span>  console.log(\`Posición del click: (\${click.x}, \${click.y})\`);</span></span>
<span class="line"><span>});</span></span></code></pre></div><h2 id="🔄-skipuntil-la-diferencia-entre-takeuntil-combinacion-de" tabindex="-1">🔄 skipUntil La diferencia entre takeUntil Combinación de <a class="header-anchor" href="#🔄-skipuntil-la-diferencia-entre-takeuntil-combinacion-de" aria-label="Permalink to &quot;🔄 skipUntil La diferencia entre takeUntil Combinación de&quot;">​</a></h2><p>Combine ambos si sólo desea obtener valores para un periodo de tiempo específico.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil, takeUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const fuente$ = intervalo(500);</span></span>
<span class="line"><span>const start$ = timer(2000); // comienza después de 2 segundos</span></span>
<span class="line"><span>const stop$ = timer(5000); // para después de 5 segundos</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fuente$.pipe(</span></span>
<span class="line"><span>  skipUntil(start$), // saltar hasta después de 2 segundos</span></span>
<span class="line"><span>  takeUntil(stop$); // parar después de 5 segundos</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Salida: 4, 5, 6, 7, 8, 9, completo.</span></span>
<span class="line"><span>// (sólo se recuperan valores entre 2 y 5 segundos)</span></span></code></pre></div><p><strong>Líneas temporales</strong>:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>0s 1s 2s 3s 4s 5s</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { interval, timer } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(500); // 0.5Emitir valor cada segundo</span></span>
<span class="line"><span>const notifier$ = timer(2000); // 2Emitir valor cada segundo</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(notifier$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// Salida: 4, 5, 6, 7, 8, ...</span></span>
<span class="line"><span>// (primer2segundo valor 0, 1, 2, 3 se omiten)</span></span></code></pre></div><p>0 1 2 3 4 5 6 7 8 9 10 ↑ arriba arriba arriba arriba arriba SKIP inicio TAKE fin (desde 4) (hasta 9)</p><h2 id="⚠️-un-error-comun" tabindex="-1">⚠️ Un error común <a class="header-anchor" href="#⚠️-un-error-comun" aria-label="Permalink to &quot;⚠️ Un error común&quot;">​</a></h2><div class="important custom-block github-alert"><p class="custom-block-title">IMPORTANT</p><p><code>skipUntil</code> son las notificaciones Observable del<strong>Sólo el primer disparo</strong>es válida.2El segundo y posteriores disparos son ignorados.</p></div><h3 id="falso-notificacionobservablese-dispara-mas-de-una-vez" tabindex="-1">Falso: NotificaciónObservablese dispara más de una vez. <a class="header-anchor" href="#falso-notificacionobservablese-dispara-mas-de-una-vez" aria-label="Permalink to &quot;Falso: NotificaciónObservablese dispara más de una vez.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { interval, Subject } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const fuente$ = interval(500);</span></span>
<span class="line"><span>const notificador$ = new Subject();</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(notificador$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Mal ejemplo: llamar a next varias veces, pero sólo la primera tiene efecto.</span></span>
<span class="line"><span>setTimeout(() =&gt; notifier$.next(), 1000);</span></span>
<span class="line"><span>setTimeout(() =&gt; notificador$.next(), 2000); // esto no tiene sentido</span></span></code></pre></div><h3 id="correcto-entiendase-que-solo-es-valido-el-primer-disparo" tabindex="-1">Correcto.: Entiéndase que sólo es válido el primer disparo <a class="header-anchor" href="#correcto-entiendase-que-solo-es-valido-el-primer-disparo" aria-label="Permalink to &quot;Correcto.: Entiéndase que sólo es válido el primer disparo&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval, Subject } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { skipUntil } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const fuente$ = interval(500);</span></span>
<span class="line"><span>const notificador$ = new Subject();</span></span>
<span class="line"><span></span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  skipUntil(notificador$)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Buen ejemplo: llamar a next sólo una vez</span></span>
<span class="line"><span>setTimeout(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Fin del salto&#39;);</span></span>
<span class="line"><span>  notifier$.next();</span></span>
<span class="line"><span>  notifier$.complete(); // buena práctica para completar.</span></span>
<span class="line"><span>}, 1000);</span></span></code></pre></div><h2 id="🎓-resumen" tabindex="-1">🎓 Resumen <a class="header-anchor" href="#🎓-resumen" aria-label="Permalink to &quot;🎓 Resumen&quot;">​</a></h2><h3 id="cuando-debe-usarse-skipuntil" tabindex="-1">Cuándo debe usarse skipUntil. <a class="header-anchor" href="#cuando-debe-usarse-skipuntil" aria-label="Permalink to &quot;Cuándo debe usarse skipUntil.&quot;">​</a></h3><ul><li>✅ Si desea iniciar el procesamiento después de que se produzca un evento específico.</li><li>✅ Si desea habilitar las operaciones de usuario después de la inicialización se ha completado</li><li>✅ Si necesita un inicio diferido en función del tiempo</li><li>✅ Si desea iniciar el procesamiento de datos una vez completada la autenticación</li></ul><h3 id="en-combinacion-con-takeuntil" tabindex="-1">En combinación con takeUntil. <a class="header-anchor" href="#en-combinacion-con-takeuntil" aria-label="Permalink to &quot;En combinación con takeUntil.&quot;">​</a></h3><ul><li>✅ Si desea obtener valores sólo durante un período de tiempo específico (skipUntil + takeUntil).</li></ul><h3 id="notas" tabindex="-1">Notas. <a class="header-anchor" href="#notas" aria-label="Permalink to &quot;Notas.&quot;">​</a></h3><ul><li>⚠️ Sólo es válido el primer disparo del Observable.</li><li>⚠️ Si el Observable no se dispara, todos los valores seguirán saltando</li><li>⚠️ La suscripción se mantiene hasta que se completa el flujo original</li></ul><h2 id="🚀-proximos-pasos" tabindex="-1">🚀 Próximos pasos. <a class="header-anchor" href="#🚀-proximos-pasos" aria-label="Permalink to &quot;🚀 Próximos pasos.&quot;">​</a></h2><ul><li><strong><a href="./skip">skip</a></strong> - aprende a saltar los N primeros valores.</li><li><strong><a href="./take">take</a></strong> - aprende a obtener los N primeros valores.</li><li><strong><a href="./../utility/takeUntil">takeUntil</a></strong> - aprende a tomar valores hasta que se dispara otro Observable.</li><li>filter](./filter)** - aprende a filtrar en base a condiciones</li><li><strong><a href="./practical-use-cases">filtering-operator-practical-use-cases</a></strong> - aprende casos de uso reales</li></ul>`,46)])])}const E=a(e,[["render",l]]);export{c as __pageData,E as default};
