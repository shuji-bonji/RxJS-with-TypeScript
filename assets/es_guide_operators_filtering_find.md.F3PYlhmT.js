import{_ as a,o as n,c as i,a2 as p}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse('{"title":"find - encuentra el primer valor que satisface la condición","description":"find es un operador de filtrado de RxJS que encuentra el primer valor que satisface una condición y lo emite, completando el flujo inmediatamente. Es ideal para situaciones en las que se desea encontrar un elemento específico de una matriz o lista, como la búsqueda de usuarios, la comprobación del inventario o la detección de registros de errores. Si no se encuentra ningún valor, devuelve undefined y en TypeScript el valor de retorno es de tipo T | undefined.","frontmatter":{"description":"find es un operador de filtrado de RxJS que encuentra el primer valor que satisface una condición y lo emite, completando el flujo inmediatamente. Es ideal para situaciones en las que se desea encontrar un elemento específico de una matriz o lista, como la búsqueda de usuarios, la comprobación del inventario o la detección de registros de errores. Si no se encuentra ningún valor, devuelve undefined y en TypeScript el valor de retorno es de tipo T | undefined."},"headers":[],"relativePath":"es/guide/operators/filtering/find.md","filePath":"es/guide/operators/filtering/find.md","lastUpdated":1779065751000}'),e={name:"es/guide/operators/filtering/find.md"};function l(t,s,r,h,k,d){return n(),i("div",null,[...s[0]||(s[0]=[p(`<h1 id="find-encuentra-el-primer-valor-que-satisface-la-condicion" tabindex="-1">find - encuentra el primer valor que satisface la condición <a class="header-anchor" href="#find-encuentra-el-primer-valor-que-satisface-la-condicion" aria-label="Permalink to &quot;find - encuentra el primer valor que satisface la condición&quot;">​</a></h1><p>El operador <code>find</code> encuentra y emite el <strong>primer valor que satisface la condición</strong> y completa el flujo inmediatamente. Si no encuentra ningún valor, devuelve <code>undefined</code>.</p><h2 id="🔰-sintaxis-basica-y-uso" tabindex="-1">🔰 Sintaxis básica y uso <a class="header-anchor" href="#🔰-sintaxis-basica-y-uso" aria-label="Permalink to &quot;🔰 Sintaxis básica y uso&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">%</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> ===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida.: 8(primer número par)</span></span></code></pre></div><p><strong>Flujo de operación</strong>:.</p><ol><li>comprobar 1, 3, 5, 7 → condición no cumplida.</li><li>comprobación 8 → condición cumplida → salida 8 y completa</li><li>9, 10 no evaluados</li></ol><p><a href="https://rxjs.dev/api/operators/find" target="_blank" rel="noreferrer">🌐 Documentación oficial de RxJS - <code>find</code></a></p><h2 id="🆚-contraste-con-first" tabindex="-1">🆚 Contraste con first <a class="header-anchor" href="#🆚-contraste-con-first" aria-label="Permalink to &quot;🆚 Contraste con first&quot;">​</a></h2><p><code>find</code> y <code>first</code> son similares, pero su uso es diferente.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find, first } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// first: Primer valor que cumple la condición (la condición es opcional)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  first</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida.: 7</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// find: Primer valor que cumple la condición (la condición es obligatoria)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida.: 7</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operador.</th><th>Especificación de la condición</th><th>Si no se encuentra ningún valor</th><th>Caso de uso.</th></tr></thead><tbody><tr><td>first()\` (primero)</td><td>Opción</td><td>Error (<code>EmptyError</code>)</td><td>Obtener el primer valor</td></tr><tr><td>first(predicado)\`</td><td>Opcional</td><td>Error (<code>EmptyError</code>)</td><td>Obtener condicional.</td></tr><tr><td>Encontrar(predicado)\`</td><td>Obligatorio.</td><td>Salida <code>undefined</code>.</td><td>Búsqueda y comprobación de existencia</td></tr></tbody></table><h2 id="💡-patron-tipico-de-utilizacion" tabindex="-1">💡 Patrón típico de utilización <a class="header-anchor" href="#💡-patron-tipico-de-utilizacion" aria-label="Permalink to &quot;💡 Patrón típico de utilización&quot;">​</a></h2><ol><li><strong>Búsqueda de usuarios</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> User</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">     id</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">     name</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">     email</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   }</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> users$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     { id: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Alice&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, email: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;alice@example.com&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     { id: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Bob&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, email: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;bob@example.com&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     { id: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Charlie&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, email: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;charlie@example.com&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> User</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // ID(la condición es opcional)2Buscar usuarios con</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   users$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> user.id </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">     if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (user) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Encontrado: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">user</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Usuario no encontrado&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Salida.: Encontrado: Bob</span></span></code></pre></div><ol start="2"><li><p><strong>Comprobación de inventario</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  id</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  name</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  stock</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> products$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A1&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;PortátilPC&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A2&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Ratón&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">15</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A3&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Teclados&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Averigüe lo que está agotado</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">products$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> product.stock </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (product) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Agotado: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">product</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Todo en stock&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida.: Agotado: PortátilPC</span></span></code></pre></div></li><li><p><strong>Buscar registro de errores</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> LogEntry</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  timestamp</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  level</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> |</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;warn&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> |</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  message</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> logs$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;App started&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;User logged in&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;error&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Connection failed&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">4</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Retry successful&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> LogEntry</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Buscar el primer error</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">logs$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> log.level </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (log) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Detección de errores: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">message</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">} (Hora: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">timestamp</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">})\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Salida.: Detección de errores: Connection failed (Hora: 3)</span></span></code></pre></div></li></ol><h2 id="🧠-ejemplo-practico-de-codigo-busqueda-de-productos" tabindex="-1">🧠 Ejemplo práctico de código (búsqueda de productos) <a class="header-anchor" href="#🧠-ejemplo-practico-de-codigo-busqueda-de-productos" aria-label="Permalink to &quot;🧠 Ejemplo práctico de código (búsqueda de productos)&quot;">​</a></h2><p>Este es un ejemplo de búsqueda de productos que coincidan con criterios específicos del stock.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaz Producto {</span></span>
<span class="line"><span>  id: cadena;</span></span>
<span class="line"><span>  nombre: string</span></span>
<span class="line"><span>  precio: número</span></span>
<span class="line"><span>  categoría: cadena;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const productos: Producto[] = [</span></span>
<span class="line"><span>  { id: &#39;P1&#39;, nombre: &#39;Ratón inalámbrico&#39;, precio: 2980, categoría: &#39;Periféricos PC&#39; }</span></span>
<span class="line"><span>  { id: &#39;P2&#39;, name: &#39;Teclado mecánico&#39;, price: 8980, category: &#39;Periféricos PC&#39; }</span></span>
<span class="line"><span>  { id: &#39;P3&#39;, name: &#39;Memoria USB 64GB&#39;, price: 1480, category: &#39;Almacenamiento&#39; }</span></span>
<span class="line"><span>  { id: &#39;P4&#39;, name: &#39;Monitor 27-inch&#39;, price: 29800, category: &#39;Displays&#39; }</span></span>
<span class="line"><span>  { id: &#39;P5&#39;, name: &#39;laptop stand&#39;, price: 3980, category: &#39;PC peripherals&#39; }</span></span>
<span class="line"><span>];</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Creación de elementos de interfaz</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(contenedor);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const title = document.createElement(&#39;h3&#39;);</span></span>
<span class="line"><span>title.textContent = &#39;Búsqueda de productos&#39;;</span></span>
<span class="line"><span>container.appendChild(title);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.type = &#39;número&#39;;</span></span>
<span class="line"><span>input.placeholder = &#39;Introduzca el precio máximo&#39;;</span></span>
<span class="line"><span>input.style.marginRight = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const searchButton = document.createElement(&#39;button&#39;);</span></span>
<span class="line"><span>searchButton.textContent = &#39;buscar&#39;;</span></span>
<span class="line"><span>container.appendChild(searchButton);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const result = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>result.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(resultado);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Proceso de búsqueda</span></span>
<span class="line"><span>// Nota: aunque originalmente el patrón recomendado es aplanar con un switchMap, pero,</span></span>
<span class="line"><span>// Nota: aunque el patrón recomendado es aplanar con un switchMap,</span><span> // aquí anidamos el subscribe por legibilidad,</span><span> // porque incluye validación UI (early return).</span></span>
<span class="line"><span>// Considera una implementación plana usando switchMap en código de producción.</span></span>
<span class="line"><span>fromEvent(searchButton, &#39;click&#39;).subscribe(() =&gt; {</span></span>
<span class="line"><span>  const maxPrice = parseInt(input.value);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  if (isNaN(maxPrice)) {</span></span>
<span class="line"><span>    result.textContent = &#39;Por favor, introduzca un precio&#39;;</span></span>
<span class="line"><span>    result.style.colour = &#39;rojo&#39;;</span></span>
<span class="line"><span>    return;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Nest subscribe: originalmente se recomendaba aplanar con switchMap</span></span>
<span class="line"><span>  from(productos).pipe(</span></span>
<span class="line"><span>    find(producto =&gt; producto.precio &lt;= preciomáximo)</span></span>
<span class="line"><span>  ).subscribe(producto =&gt; {</span></span>
<span class="line"><span>    if (producto) {</span></span>
<span class="line"><span>      result.innerHTML = \`</span></span>
<span class="line"><span>        &lt;strong&gt;¡Encontrado! &lt;/strong&gt;&lt;br&gt;</span></span>
<span class="line"><span>        Nombre del producto: \${product.name}&lt;br&gt;</span></span>
<span class="line"><span>        Precio: \${product.price.toLocaleString()}&lt;br&gt;</span></span>
<span class="line"><span>        Categoría: \${product.category}</span></span>
<span class="line"><span>      \`;</span></span>
<span class="line"><span>      result.style.color = &#39;verde&#39;;</span></span>
<span class="line"><span>    } else {</span></span>
<span class="line"><span>      result.textContent = \`¥{maxPrice.toLocaleString()} o menos producto no encontrado \`;</span></span>
<span class="line"><span>      result.style.color = &#39;orange&#39;; }</span></span>
<span class="line"><span>    }</span></span>
<span class="line"><span>  });</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Este código busca y muestra el primer producto por debajo del precio introducido por el usuario.</p><h2 id="🎯-filter-la-diferencia-entre" tabindex="-1">🎯 filter La diferencia entre <a class="header-anchor" href="#🎯-filter-la-diferencia-entre" aria-label="Permalink to &quot;🎯 filter La diferencia entre&quot;">​</a></h2><p><code>find</code> y <code>filter</code> se utilizan con fines distintos.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, filter } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const números$ = from([1, 3, 5, 7, 8, 9, 10]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// filter: muestra todos los valores que cumplen la condición</span></span>
<span class="line"><span>números$.pipe(</span></span>
<span class="line"><span>  filter(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;filtro complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Salida: 7, 8, 9, 10, filter completo</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: salida sólo el primer valor que coincida con la condición</span></span>
<span class="line"><span>números$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;find complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// salida: 7, find completo</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operador</th><th>Número de salidas</th><th>Tiempo de realización</th><th>Caso de uso</th></tr></thead><tbody><tr><td><code>filter(predicate)</code></td><td>Todos los valores que cumplen la condición</td><td>Al finalizar el flujo original</td><td>Depuración de datos</td></tr><tr><td><code>find(predicate)</code></td><td>Sólo el primer valor que cumpla los criterios</td><td>Inmediatamente después del descubrimiento</td><td>Búsqueda y comprobación de existencia</td></tr></tbody></table><h2 id="📋-uso-seguro" tabindex="-1">📋 Uso seguro <a class="header-anchor" href="#📋-uso-seguro" aria-label="Permalink to &quot;📋 Uso seguro&quot;">​</a></h2><p>TypeScript Este es un ejemplo de implementación segura que utiliza genéricos en</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaz Task {</span></span>
<span class="line"><span>  id: número;</span></span>
<span class="line"><span>  título: string</span></span>
<span class="line"><span>  completed: booleano;</span></span>
<span class="line"><span>  prioridad: &#39;alta&#39; | &#39;media&#39; | &#39;baja&#39;; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findTaskById(</span></span>
<span class="line"><span>  tareas$: Observable\\&lt;Tarea&gt;,.</span></span>
<span class="line"><span>  id: número</span></span>
<span class="line"><span>): Observable&lt;Tarea | undefined&gt; {</span></span>
<span class="line"><span>  return tareas$.pipe(</span></span>
<span class="line"><span>    find(tarea =&gt; tarea.id === id)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>función findFirstIncompleteTask(</span></span>
<span class="line"><span>  tareas$: Observable\\&lt;Tarea&gt;</span></span>
<span class="line"><span>): Observable&lt;Tarea | undefined&gt; {</span></span>
<span class="line"><span>  return tareas$.pipe(</span></span>
<span class="line"><span>    find(tarea =&gt; !tarea.complete)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Ejemplo de uso</span></span>
<span class="line"><span>const tareas$ = from([.</span></span>
<span class="line"><span>  { id: 1, title: &#39;Tarea A&#39;, completed: true, priority: &#39;high&#39; as const }</span></span>
<span class="line"><span>  { id: 2, title: &#39;Tarea B&#39;, complete: false, prioridad: &#39;media&#39; as const }</span></span>
<span class="line"><span>  { id: 3, title: &#39;Tarea C&#39;, completada: false, prioridad: &#39;baja&#39; as const }</span></span>
<span class="line"><span>] as Task[]);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Búsqueda por ID</span></span>
<span class="line"><span>findTaskById(tareas$, 2).subscribe(tarea =&gt; {</span></span>
<span class="line"><span>  if (tarea) {</span></span>
<span class="line"><span>    console.log(\`encontrado: \${tarea.titulo}\`);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Tarea no encontrada&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Salida: encontrada: tarea B</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Encontrar tareas no completadas</span></span>
<span class="line"><span>findFirstIncompleteTask(tareas$).subscribe(tarea =&gt; {</span></span>
<span class="line"><span>  if (tarea) {</span></span>
<span class="line"><span>    console.log(\`Siguiente tarea: \${task.title} (prioridad: \${task.priority})\`);</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Salida: siguiente tarea: tarea B (prioridad: media)</span></span></code></pre></div><h2 id="🔄-find-y-findindex-la-diferencia-entre" tabindex="-1">🔄 find y findIndex La diferencia entre <a class="header-anchor" href="#🔄-find-y-findindex-la-diferencia-entre" aria-label="Permalink to &quot;🔄 find y findIndex La diferencia entre&quot;">​</a></h2><p>RxJSen los operadores <code>findIndex</code> también están disponibles.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, findIndex } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const números$ = from([10, 20, 30, 40, 50]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: devuelve un valor</span></span>
<span class="line"><span>números$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// salida: 30</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// findIndex: devuelve el índice</span></span>
<span class="line"><span>números$.pipe(</span></span>
<span class="line"><span>  findIndex(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Salida: 2 (índice de 30)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operador</th><th>Devuelve el valor</th><th>si no se encuentra el valor</th></tr></thead><tbody><tr><td><code>find(predicate)</code></td><td>Valor propiamente dicho</td><td><code>undefined</code></td></tr><tr><td><code>findIndex(predicate)</code></td><td>Índice (valor numérico)</td><td><code>-1</code></td></tr></tbody></table><h2 id="⚠️-un-error-comun" tabindex="-1">⚠️ Un error común <a class="header-anchor" href="#⚠️-un-error-comun" aria-label="Permalink to &quot;⚠️ Un error común&quot;">​</a></h2><div class="note custom-block github-alert"><p class="custom-block-title">NOTE</p><p><code>find</code> si no se encuentra el valor <code>undefined</code> es emitido. Esto no da lugar a un error. Si se requiere un error, utilice <code>first</code> para ser utilizado.</p></div><h3 id="error-tratamiento-del-error-esperado-si-no-se-encuentra-el-valor" tabindex="-1">Error.: Tratamiento del error esperado si no se encuentra el valor. <a class="header-anchor" href="#error-tratamiento-del-error-esperado-si-no-se-encuentra-el-valor" aria-label="Permalink to &quot;Error.: Tratamiento del error esperado si no se encuentra el valor.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const números$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Mal ejemplo: gestión de errores esperada pero no invocada</span></span>
<span class="line"><span>números$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err) // no llamado</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// salida: indefinida</span></span></code></pre></div><h3 id="positivo-undefined-comprobar-o-first-utilizar-el" tabindex="-1">Positivo: undefined Comprobar o first utilizar el <a class="header-anchor" href="#positivo-undefined-comprobar-o-first-utilizar-el" aria-label="Permalink to &quot;Positivo: undefined Comprobar o first utilizar el&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, first } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const números$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Buen ejemplo 1: comprobación de indefinidos</span></span>
<span class="line"><span>números$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe(result =&gt; {</span></span>
<span class="line"><span>  ¡if (resultado ! == undefined) {</span></span>
<span class="line"><span>    console.log(&#39;Encontrado:&#39;, resultado);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;No encontrado:&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Salida: no encontrado</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Buen ejemplo 2: usa first si necesitas un error</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  first(n =&gt; n &gt; 10, 0) // especificar valor por defecto</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err.message)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Salida: 0</span></span></code></pre></div><h2 id="🎓-resumen" tabindex="-1">🎓 Resumen <a class="header-anchor" href="#🎓-resumen" aria-label="Permalink to &quot;🎓 Resumen&quot;">​</a></h2><h3 id="cuando-debes-usar-find" tabindex="-1">Cuándo debes usar find. <a class="header-anchor" href="#cuando-debes-usar-find" aria-label="Permalink to &quot;Cuándo debes usar find.&quot;">​</a></h3><ul><li>✅ Si quieres encontrar el primer valor que satisface una condición.</li><li>✅ Cuando quieras comprobar la existencia de un valor.</li><li>✅ Cuando se desea tratar un valor como <code>undefined</code> si no se encuentra.</li><li>✅ Cuando se desea encontrar un elemento concreto en una matriz o lista.</li></ul><h3 id="cuando-debe-utilizar-first" tabindex="-1">Cuando debe utilizar first. <a class="header-anchor" href="#cuando-debe-utilizar-first" aria-label="Permalink to &quot;Cuando debe utilizar first.&quot;">​</a></h3><ul><li>✅ Si quieres obtener el primer valor.</li><li>✅ Si desea emitir un error si no se encuentra el valor</li></ul><h3 id="¿cuando-se-debe-utilizar-filter" tabindex="-1">¿Cuándo se debe utilizar filter? <a class="header-anchor" href="#¿cuando-se-debe-utilizar-filter" aria-label="Permalink to &quot;¿Cuándo se debe utilizar filter?&quot;">​</a></h3><ul><li>✅ Si necesita todos los valores que coincidan con una condición</li><li>✅ Si desea filtrar los datos</li></ul><h3 id="notas" tabindex="-1">Notas. <a class="header-anchor" href="#notas" aria-label="Permalink to &quot;Notas.&quot;">​</a></h3><ul><li>⚠️ <code>find</code> da como resultado <code>undefined</code> si no se encuentra (no es un error)</li><li>⚠️ Completa inmediatamente con el primer valor que satisface la condición</li><li>⚠️ TypeScript devuelve un valor de tipo <code>T | undefined</code>.</li></ul><h2 id="🚀-siguiente-paso" tabindex="-1">🚀 Siguiente paso. <a class="header-anchor" href="#🚀-siguiente-paso" aria-label="Permalink to &quot;🚀 Siguiente paso.&quot;">​</a></h2><ul><li><strong>[first](. /first)</strong> - aprende a obtener el primer valor.</li><li><strong>[filter](. /filter)</strong> - aprende a filtrar en base a condiciones.</li><li>findIndex](<a href="https://rxjs.dev/api/operators/findIndex" target="_blank" rel="noreferrer">https://rxjs.dev/api/operators/findIndex</a>)** - aprende a obtener el índice del primer valor que satisface una condición (documentación oficial).</li><li><strong>[filtro-operador-casos-prácticos](. /practical-use-cases)</strong> - aprende casos de uso reales</li></ul>`,47)])])}const E=a(e,[["render",l]]);export{c as __pageData,E as default};
