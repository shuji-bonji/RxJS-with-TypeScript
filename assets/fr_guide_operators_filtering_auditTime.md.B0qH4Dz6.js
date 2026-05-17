import{_ as a,o as i,c as n,a2 as e}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse(`{"title":"auditTime - dernière valeur émise après le temps spécifié","description":"auditTime est un opérateur de filtrage de RxJS qui attend un temps spécifié lorsqu'une valeur est émise et produit la dernière valeur au cours de cette période. Il est préférable de l'utiliser lorsque vous souhaitez échantillonner périodiquement le dernier état sur des événements à haute fréquence tels que le suivi de la position du défilement, le redimensionnement de la fenêtre, le mouvement de la souris, etc. Il est important de comprendre la différence entre cet opérateur et throttleTime et debounceTime et de les utiliser de manière appropriée.","frontmatter":{"description":"auditTime est un opérateur de filtrage de RxJS qui attend un temps spécifié lorsqu'une valeur est émise et produit la dernière valeur au cours de cette période. Il est préférable de l'utiliser lorsque vous souhaitez échantillonner périodiquement le dernier état sur des événements à haute fréquence tels que le suivi de la position du défilement, le redimensionnement de la fenêtre, le mouvement de la souris, etc. Il est important de comprendre la différence entre cet opérateur et throttleTime et debounceTime et de les utiliser de manière appropriée."},"headers":[],"relativePath":"fr/guide/operators/filtering/auditTime.md","filePath":"fr/guide/operators/filtering/auditTime.md","lastUpdated":1779057779000}`),p={name:"fr/guide/operators/filtering/auditTime.md"};function l(t,s,h,r,k,d){return i(),n("div",null,[...s[0]||(s[0]=[e(`<h1 id="audittime-derniere-valeur-emise-apres-le-temps-specifie" tabindex="-1">auditTime - dernière valeur émise après le temps spécifié <a class="header-anchor" href="#audittime-derniere-valeur-emise-apres-le-temps-specifie" aria-label="Permalink to &quot;auditTime - dernière valeur émise après le temps spécifié&quot;">​</a></h1><p>L&#39;opérateur <code>auditTime</code> attend un <strong>temps spécifié</strong> après l&#39;émission d&#39;une valeur et produit la <strong>dernière valeur</strong> pendant cette période. Il attend ensuite l&#39;arrivée de la valeur suivante.</p><h2 id="🔰-syntaxe-de-base-et-utilisation" tabindex="-1">🔰 Syntaxe de base et utilisation <a class="header-anchor" href="#🔰-syntaxe-de-base-et-utilisation" aria-label="Permalink to &quot;🔰 Syntaxe de base et utilisation&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Cliquez.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span></code></pre></div><p><strong>Flux des opérations</strong> :.</p><ol><li>le premier clic se produit</li><li>attendre 1 seconde (les clics pendant ce temps sont enregistrés mais ne sont pas émis)</li><li>sortie du dernier clic après 1 seconde Attendre le clic suivant</li></ol><p><a href="https://rxjs.dev/api/operators/auditTime" target="_blank" rel="noreferrer">🌐 RxJS official documentation - <code>auditTime</code></a></p><h2 id="🆚-contraste-avec-throttletime" tabindex="-1">🆚 Contraste avec throttleTime <a class="header-anchor" href="#🆚-contraste-avec-throttletime" aria-label="Permalink to &quot;🆚 Contraste avec throttleTime&quot;">​</a></h2><p><code>throttleTime</code> et <code>auditTime</code> sont similaires, mais diffèrent dans les valeurs qu&#39;ils produisent.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { interval } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { throttleTime, auditTime, take } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> source$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> interval</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">300</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">take</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)); </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// throttleTime: Sortie de la première valeur</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  throttleTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: 0, 4, 8(première valeur de chaque période)</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// auditTime: Sortie de la dernière valeur</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">source$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: 3, 6, 9(dernière valeur de chaque période)</span></span></code></pre></div><p><strong>Comparaison de lignes de temps</strong> :.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>Source:     0--1--2--3--4--5--6--7--8--9--|</span></span>
<span class="line"><span>            |        |        |</span></span>
<span class="line"><span>throttle:   0--------4--------8------------|</span></span>
<span class="line"><span>            (Première)   (Première)   (Première)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>audit:      -------3--------6--------9----|</span></span>
<span class="line"><span>                  (Dernière)   (Dernière)   (Dernière)</span></span></code></pre></div><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(document, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;click&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1000</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Cliquez.！&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">));</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`\`\`ts</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">import { interval } from &#39;rxjs&#39;;</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">import { throttleTime, auditTime, take } from &#39;rxjs&#39;;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// throttleTime: PremierのvaleurをSortie</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">source$.pipe(</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">  throttleTime(1000)</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">).subscribe(console.log);</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// Sortie: 0, 4, 8（各périodeのPremierのvaleur）</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// auditTime: DernierのvaleurをSortie</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">source$.pipe(</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">  auditTime(1000)</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">).subscribe(console.log);</span></span>
<span class="line"><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">// Sortie: 3, 6, 9（各périodeのDernierのvaleur）</span></span></code></pre></div><h2 id="💡-modele-d-utilisation-typique" tabindex="-1">💡 Modèle d&#39;utilisation typique <a class="header-anchor" href="#💡-modele-d-utilisation-typique" aria-label="Permalink to &quot;💡 Modèle d&#39;utilisation typique&quot;">​</a></h2><ol><li><strong>Optimiser le redimensionnement des fenêtres</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">   fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;resize&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">200</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">) </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// 200msObtenir la dernière taille dans l&#39;intervalle</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Taille de la fenêtre: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerWidth</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}x\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">window</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">innerHeight</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span></code></pre></div><ol start="2"><li><p><strong>Suivi de la position de défilement</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">fromEvent</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(window, </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;scroll&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">100</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">),</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  map</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(() </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> ({</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollY: window.scrollY,</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    scrollX: window.scrollX</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }))</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">position</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Position du défilement: Y=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollY</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}, X=\${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">position</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">scrollX</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li><li><p><strong>Mouvement de glissement en douceur</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { fromEvent } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { auditTime, map, takeUntil, switchMap } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Créer des éléments pouvant être glissés</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> box</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> document.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">createElement</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;div&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.width </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.height </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.backgroundColor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;#3498db&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.position </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;absolute&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.cursor </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;move&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.left </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.style.top </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;100px&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">box.textContent </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">=</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;Glissement&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Mettre en œuvre des opérations de glissement</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">mouseDown$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  switchMap</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">startEvent</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startX</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientX </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetLeft;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> startY</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> startEvent.clientY </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">-</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> box.offsetTop;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">    return</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> mouseMove$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">      auditTime</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">16</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">), </span><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Approximativement.60FPS(voir aussi16ms) pour mettre à jour la position</span></span>
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
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span></code></pre></div></li></ol><h2 id="🧠-exemple-de-code-pratique-suivi-de-la-souris" tabindex="-1">🧠 Exemple de code pratique (suivi de la souris) <a class="header-anchor" href="#🧠-exemple-de-code-pratique-suivi-de-la-souris" aria-label="Permalink to &quot;🧠 Exemple de code pratique (suivi de la souris)&quot;">​</a></h2><p>Cet exemple suit les mouvements de la souris et affiche la dernière position à intervalles réguliers.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { auditTime, map } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Création d&#39;éléments d&#39;interface utilisateur</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;) ;.</span></span>
<span class="line"><span>container.style.height = &#39;300px&#39; ;</span></span>
<span class="line"><span>container.style.border = &#39;2px solid #3498db&#39; ;</span></span>
<span class="line"><span>container.style.padding = &#39;20px&#39; ;</span></span>
<span class="line"><span>container.style.position = &#39;relative&#39; ;</span></span>
<span class="line"><span>container.textContent = &quot;Veuillez déplacer la souris dans cette zone&quot; ;</span></span>
<span class="line"><span>document.body.appendChild(container) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const positionDisplay = document.createElement(&#39;div&#39;) ;</span></span>
<span class="line"><span>positionDisplay.style.marginTop = &#39;10px&#39; ;</span></span>
<span class="line"><span>positionDisplay.style.fontFamily = &#39;monospace&#39; ;</span></span>
<span class="line"><span>document.body.appendChild(positionDisplay) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const dot = document.createElement(&#39;div&#39;) ;</span></span>
<span class="line"><span>dot.style.width = &#39;10px&#39; ;</span></span>
<span class="line"><span>dot.style.height = &#39;10px&#39; ;</span></span>
<span class="line"><span>dot.style.borderRadius = &#39;50%&#39; ;</span></span>
<span class="line"><span>dot.style.backgroundColor = &#39;#e74c3c&#39; ;</span></span>
<span class="line"><span>dot.style.position = &#39;absolute&#39; ;</span></span>
<span class="line"><span>dot.style.display = &#39;none&#39; ;</span></span>
<span class="line"><span>container.appendChild(dot) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Événement de déplacement de la souris</span></span>
<span class="line"><span>fromEvent&lt;MouseEvent&gt;(container, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>  map(event =&gt; {</span></span>
<span class="line"><span>    const rect = container.getBoundingClientRect() ;</span></span>
<span class="line"><span>    return {</span></span>
<span class="line"><span>      x : event.clientX - rect.left,.</span></span>
<span class="line"><span>      y : event.clientY - rect.top</span></span>
<span class="line"><span>    } ;</span></span>
<span class="line"><span>  }),</span></span>
<span class="line"><span>  auditTime(100) // Récupère la dernière position toutes les 100ms</span></span>
<span class="line"><span>).subscribe(position =&gt; {</span></span>
<span class="line"><span>  positionDisplay.textContent = \`Dernière position (toutes les 100ms) : X=\${position.x.toFixed(0)}, Y=\${position.y.toFixed(0)}\` ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Déplacer le point vers la dernière position</span></span>
<span class="line"><span>  dot.style.left = \`\${position.x - 5}px\` ;</span></span>
<span class="line"><span>  dot.style.top = \`\${position.y - 5}px\` ;</span></span>
<span class="line"><span>  dot.style.display = &#39;block&#39; ;</span></span>
<span class="line"><span>}) ;</span></span></code></pre></div><p>Ce code ne récupère et n&#39;affiche la dernière position qu&#39;à chaque fois que la souris est déplacée, même si la souris est déplacée fréquemment,100msLe code ne récupère et n&#39;affiche la dernière position que pour chaque mouvement de la souris.</p><h2 id="🎯-debouncetime-differences-entre" tabindex="-1">🎯 debounceTime Différences entre <a class="header-anchor" href="#🎯-debouncetime-differences-entre" aria-label="Permalink to &quot;🎯 debounceTime Différences entre&quot;">​</a></h2><p><code>auditTime</code> et <code>debounceTime</code> est que<strong>affichent tous deux la dernière valeur</strong>mais le code<strong>Le timing est complètement différent</strong>la dernière valeur est émise.</p><h3 id="la-difference-decisive" tabindex="-1">La différence décisive <a class="header-anchor" href="#la-difference-decisive" aria-label="Permalink to &quot;La différence décisive&quot;">​</a></h3><table tabindex="0"><thead><tr><th>L&#39;opérateur</th><th>l&#39;opération</th><th>utilisation différente du système</th></tr></thead><tbody><tr><td><code>auditTime(ms)</code></td><td>À l&#39;arrivée d&#39;une valeur<strong>msToujours éditer après</strong>(même si l&#39;entrée se poursuit)</td><td>Échantillonnage à intervalles réguliers</td></tr><tr><td><code>debounceTime(ms)</code></td><td><strong>Après l&#39;arrêt de l&#39;entrée</strong>msSortie après</td><td>Attendre la fin de l&#39;entrée</td></tr></tbody></table><h3 id="exemples-specifiques-differences-dans-l-entree-de-la-recherche" tabindex="-1">Exemples spécifiques：Différences dans l&#39;entrée de la recherche <a class="header-anchor" href="#exemples-specifiques-differences-dans-l-entree-de-la-recherche" aria-label="Permalink to &quot;Exemples spécifiques：Différences dans l&#39;entrée de la recherche&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { auditTime, debounceTime } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;) ;</span></span>
<span class="line"><span>input.placeholder = &#39;Search word input&#39; ;</span></span>
<span class="line"><span>document.body.appendChild(input) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// auditTime : Exécution de la recherche toutes les 300 ms même pendant la saisie</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300)</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;auditTime → Search:&#39;, input.value) ;</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// debounceTime : attendre 300ms après l&#39;arrêt de l&#39;entrée, puis exécuter la recherche</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300)</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;debounceTime → Search:&#39;, input.value) ;</span></span>
<span class="line"><span>}) ;</span></span></code></pre></div><h3 id="differences-observees-dans-la-ligne-de-temps" tabindex="-1">Différences observées dans la ligne de temps <a class="header-anchor" href="#differences-observees-dans-la-ligne-de-temps" aria-label="Permalink to &quot;Différences observées dans la ligne de temps&quot;">​</a></h3><p>Différence observée lorsqu&#39;un utilisateur clique sur &quot;ab&#39;→&#39;abc&#39;→&#39;abcd&#39; lors d&#39;une saisie rapide:</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>Événement d&#39;entrée : a--b--c--d------------|</span></span>
<span class="line"><span>              ↓</span></span>
<span class="line"><span>auditTime : ------c-----d----------|</span></span>
<span class="line"><span>            (après 300 ms) (après 300 ms)</span></span>
<span class="line"><span>            → Recherche de &#39;abc&#39;, recherche de &#39;abcd&#39; (2 fois au total)</span></span>
<span class="line"><span></span></span>
<span class="line"><span>debounceTime : --------------------d-|</span></span>
<span class="line"><span>                              (300 ms après l&#39;arrêt)</span></span>
<span class="line"><span>            → Recherche de &quot;abcd&quot; (une seule fois au total)</span></span></code></pre></div><p><strong>Facile à retenir</strong>:</p><ul><li><strong><code>auditTime</code></strong>: Régulièrement contrôlé (audit)&quot;→ &quot;Toujours vérifier à intervalles réguliers</li><li><strong><code>debounceTime</code></strong>: Attendez qu&#39;il se calme (...)&quot;.debounceAttendez que ce soit calme.→ Attendre qu&#39;il y ait du calme</li></ul><h3 id="utilisation-pratique" tabindex="-1">Utilisation pratique <a class="header-anchor" href="#utilisation-pratique" aria-label="Permalink to &quot;Utilisation pratique&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>// ✅ auditTime si nécessaire</span></span>
<span class="line"><span>// - Suivi de la position de défilement (nous voulons l&#39;obtenir périodiquement, même si nous défilons tout le temps)</span></span>
<span class="line"><span>fromEvent(window, &#39;scroll&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(100) // récupère la dernière position toutes les 100ms</span></span>
<span class="line"><span>).subscribe(/* ... */) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ si debounceTime est approprié.</span></span>
<span class="line"><span>// - boîte de recherche (nous voulons effectuer une recherche une fois la saisie terminée)</span></span>
<span class="line"><span>fromEvent(searchInput, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  debounceTime(300) // attend 300ms après l&#39;arrêt de la saisie</span></span>
<span class="line"><span>).subscribe(/* ... */) ;</span></span></code></pre></div><h2 id="📋-utilisation-sure-du-point-de-vue-du-type" tabindex="-1">📋 Utilisation sûre du point de vue du type <a class="header-anchor" href="#📋-utilisation-sure-du-point-de-vue-du-type" aria-label="Permalink to &quot;📋 Utilisation sûre du point de vue du type&quot;">​</a></h2><p>TypeScript Il s&#39;agit d&#39;un exemple d&#39;implémentation sûre du point de vue du type, qui utilise les éléments génériques dans le cadre d&#39;un projet de développement.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, fromEvent } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { auditTime, map } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface MousePosition {</span></span>
<span class="line"><span>  x : nombre ;</span></span>
<span class="line"><span>  y : nombre ;</span></span>
<span class="line"><span>  timestamp : nombre ; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function trackMousePosition(</span></span>
<span class="line"><span>  element : HTMLElement,.</span></span>
<span class="line"><span>  interval : nombre</span></span>
<span class="line"><span>) : Observable {</span></span>
<span class="line"><span>  return fromEvent&lt;MouseEvent&gt;(element, &#39;mousemove&#39;).pipe(</span></span>
<span class="line"><span>    auditTime(intervalMs),.</span></span>
<span class="line"><span>    map(event =&gt; ({</span></span>
<span class="line"><span>      x : event.clientX, event.</span></span>
<span class="line"><span>      y : event.clientY,.</span></span>
<span class="line"><span>      timestamp : Date.now())</span></span>
<span class="line"><span>    } as MousePosition))</span></span>
<span class="line"><span>  ) ;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Exemple d&#39;utilisation</span></span>
<span class="line"><span>const canvas = document.createElement(&#39;div&#39;) ;</span></span>
<span class="line"><span>canvas.style.width = &#39;400px&#39; ;</span></span>
<span class="line"><span>canvas.style.height = &#39;300px&#39; ;</span></span>
<span class="line"><span>canvas.style.border = &#39;1px solid black&#39; ;</span></span>
<span class="line"><span>document.body.appendChild(canvas) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>trackMousePosition(canvas, 200).subscribe(position =&gt; {</span></span>
<span class="line"><span>  console.log(\`Position : (\${position.x}, \${position.y}) at \${position.timestamp}\`) ;</span></span>
<span class="line"><span>}) ;</span></span></code></pre></div><h2 id="🔄-audittime-et-throttletime-combinaison-de" tabindex="-1">🔄 auditTime et throttleTime Combinaison de <a class="header-anchor" href="#🔄-audittime-et-throttletime-combinaison-de" aria-label="Permalink to &quot;🔄 auditTime et throttleTime Combinaison de&quot;">​</a></h2><p>Dans certains cas, les deux peuvent être combinés.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { interval } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { throttleTime, auditTime, take } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const source$ = interval(100).pipe(take(50)) ;.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ordre de throttleTime → auditTime</span></span>
<span class="line"><span>source$.pipe(</span></span>
<span class="line"><span>  throttleTime(1000), // passer la première valeur toutes les secondes</span></span>
<span class="line"><span>  auditTime(500) // puis attendre 500ms et sortir la dernière valeur</span></span>
<span class="line"><span>).subscribe(console.log) ;.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Cliquez.！&#39;));</span></span>
<span class="line"><span>---</span></span>
<span class="line"><span>description: auditTimeは値が発行されたら指定時間待機し、その期間内の最後の値を出力するRxJSフィルタリングオペレーターです。スクロール位置の追跡、ウィンドウリサイズ、マウス移動などの高頻度イベントで最新の状態を定期的にサンプリングしたい場合に最適です。throttleTimeやdebounceTimeとの違いを理解して適切に使い分けることが重要です。</span></span>
<span class="line"><span>---</span></span>
<span class="line"><span></span></span>
<span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Création d&#39;un champ de saisie de recherche</span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;) ;.</span></span>
<span class="line"><span>input.type = &#39;text&#39; ;</span></span>
<span class="line"><span>input.placeholder = &#39;Rechercher...&#39; ;</span></span>
<span class="line"><span>document.body.appendChild(input) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Mauvais exemple : utilisation de l&#39;auditTime pour l&#39;entrée de recherche</span></span>
<span class="line"><span>fromEvent(input, &#39;input&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(300) // la recherche est effectuée toutes les 300ms pendant la saisie</span></span>
<span class="line"><span>).subscribe(() =&gt; {</span></span>
<span class="line"><span>  console.log(&#39;Search executed&#39;) ;</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;Cliquez.！&#39;));</span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { auditTime } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>fromEvent(document, &#39;click&#39;).pipe(</span></span>
<span class="line"><span>  auditTime(1000)</span></span>
<span class="line"><span>).subscribe(() =&gt; console.log(&#39;クリック！&#39;));</span></span></code></pre></div><p>ts. import { fromEvent } from &#39;rxjs&#39; ; import { debounceTime } from &#39;rxjs&#39; ;</p><p>// Création d&#39;un champ de saisie de recherche const input = document.createElement(&#39;input&#39;) ;. input.type = &#39;text&#39; ; input.placeholder = &#39;Rechercher...&#39; ; document.body.appendChild(input) ;</p><p>// ✅ Bon exemple : utiliser debounceTime pour une entrée de recherche fromEvent(input, &#39;input&#39;).pipe( debounceTime(300) // Attendre 300ms après l&#39;arrêt de l&#39;entrée avant d&#39;effectuer une recherche ).subscribe(() =&gt; { console.log(&#39;Search executed&#39;, input.value) ; }) ;</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>## 🎓 Résumé</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Quand auditTime doit être utilisé.</span></span>
<span class="line"><span>- ✅ Lorsque des valeurs actualisées sont requises à intervalles réguliers.</span></span>
<span class="line"><span>- ✅ Événements à haute fréquence tels que le défilement, le redimensionnement, le mouvement de la souris.</span></span>
<span class="line"><span>- ✅ Lorsqu&#39;un échantillonnage périodique est nécessaire</span></span>
<span class="line"><span>- lorsque vous souhaitez refléter l&#39;état le plus récent.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Quand throttleTime doit être utilisé .</span></span>
<span class="line"><span>- ✅ Lorsqu&#39;une réponse immédiate est nécessaire</span></span>
<span class="line"><span>- ✅ Si vous voulez commencer le traitement avec la première valeur</span></span>
<span class="line"><span>- ✅ Prévention de l&#39;enfoncement des boutons</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Quand utiliser debounceTime.</span></span>
<span class="line"><span>- ✅ Si vous voulez attendre que l&#39;entrée soit terminée</span></span>
<span class="line"><span>- ✅ Recherche, autocomplétion</span></span>
<span class="line"><span>- ✅ Attendre que l&#39;utilisateur arrête de taper.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>### Notes.</span></span>
<span class="line"><span>- ⚠️ \`auditTime\` ne produit que la dernière valeur de la période (les valeurs intermédiaires sont rejetées).</span></span>
<span class="line"><span>- ⚠️ N&#39;est pas très efficace si elle est définie pour des intervalles courts.</span></span>
<span class="line"><span>- ⚠️ \`throttleTime\` ou \`debounceTime\` peuvent être plus appropriés en fonction de l&#39;application.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>## 🚀 Prochaines étapes.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>- **[throttleTime](. /throttleTime)** - apprendre à passer la première valeur.</span></span>
<span class="line"><span>- **[debounceTime](. /debounceTime)** - apprenez à émettre des valeurs après l&#39;arrêt de la saisie.</span></span>
<span class="line"><span>- **[filter](. /filter)** - apprendre à filtrer en fonction de conditions.</span></span>
<span class="line"><span>- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - Apprenez à utiliser des cas d&#39;utilisation réels.</span></span></code></pre></div>`,44)])])}const E=a(p,[["render",l]]);export{c as __pageData,E as default};
