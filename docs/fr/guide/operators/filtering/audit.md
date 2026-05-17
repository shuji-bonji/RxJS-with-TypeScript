---
description: "L'opérateur `audit` est un opérateur de filtrage RxJS qui n'émet que la dernière valeur au cours de la période contrôlée par un Observable personnalisé. Idéal pour le contrôle dynamique du timing."
head:
  - - meta
    - name: keywords
      content: RxJS, audit, opérateur de filtrage, contrôle dynamique du timing, Observable, TypeScript
---

# audit - dernière valeur de la période de contrôle émise

L'opérateur `audit` attend qu'un Observable personnalisé émette une valeur et émet la **dernière valeur** émise par la source au cours de cette période.
Alors que `auditTime` est contrôlé par un temps fixe, `audit` permet de **contrôler la période** avec un Observable dynamique.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Événement de clic
const clicks$ = fromEvent(document, 'click');

// 1Délimiter par périodes de N secondes
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Clic enregistré');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge
- **Contrôle de timing personnalisé** : contrôle de la période basé sur d'autres Observables
- **Limitation adaptative des événements** : régulation sensible au contexte

## 🔍 Différences avec auditTime

| Opérateur | Contrôle de la période | Cas d'usage |
|---|---|---|
| `auditTime` | Temps fixe (millisecondes) | Contrôle simple basé sur le temps |
| `audit` | **Observable personnalisé** | **Contrôle dynamique de la période** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fixe1secondes
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fixe1secondes'));

// audit - Période dynamique
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~2Période aléatoire de secondes
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICréer
const output = document.createElement('div');
output.innerHTML = '<h3>Échantillonnage dynamique</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Changer la charge';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Niveau de charge (0: Faible,1: Moyen,2: Élevé)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Faible charge', 'Charge moyenne', 'Charge élevée'];
  statusDiv.textContent = `Charge actuelle: ${levels[loadLevel]}`;
});

// Événement mousemove
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Ajuster la période selon la charge
    const periods = [2000, 1000, 500]; // Faible charge→Période longue, charge élevée→Période courte
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Position de la souris: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10éléments affichés
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Échantillonnage réduit à des intervalles de 2 s lorsque la charge est faible (mode économie d'énergie)
- Échantillonnage fin à intervalles de 500 ms lorsque la charge est élevée
- La période peut être ajustée dynamiquement en fonction de la charge

## 🎯 Exemple de code pratique 2 : Contrôle de la période basé sur d'autres flux

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UICréer
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervalle: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Surveiller la valeur du slider
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Événement de clic
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Mettre à jour la valeur du slider
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Clicauditcontrôlé par
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Enregistrement de clic (intervalle: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Points d'attention

### 1. La première valeur n'est pas émise immédiatement

`audit` attend la fin de la période après avoir reçu la première valeur.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Sortie:
// 9  (1secondes après,0~9dernière valeur de)
// 19 (2secondes après,10~19dernière valeur de)
// 29 (3secondes après,20~29dernière valeur de)
```

### 2. L'Observable de durée est généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Mauvais exemple: MêmeObservableinstance réutilisée
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2ne fonctionne plus après la 2e fois
).subscribe();

// ✅ Bon exemple: Nouveau à chaque foisObservableGénère un
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

L'utilisation de `audit` sur des flux où des valeurs sont fréquemment émises consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Stream rapide (10ms chaque)
interval(10).pipe(
  audit(() => timer(1000)) // 1Échantillonnage chaque seconde
).subscribe();
// 1Pendant N secondes,100valeurs s'accumulent en mémoire et seule la dernière1est émise
```

## 🆚 Comparaison avec des opérateurs similaires

| Opérateur | Quand émettre | Valeur émise | Cas d'usage |
|---|---|---|---|
| `audit` | À la **fin** de la période | La **dernière** valeur de la période | Obtenir le dernier état dans la période |
| `throttle` | Au **début** de la période | La **première** valeur de la période | Obtenir le début d'une séquence d'événements |
| `debounce` | **Après stabilisation** | Valeur juste avant stabilisation | Attendre la fin de la saisie |
| `sample` | **Quand un autre Observable se déclenche** | Valeur la plus récente à ce moment | Snapshot périodique |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Dernier clic en N secondes
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Dernier'));

// throttle: 1Premier clic en N secondes
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Premier'));

// debounce: Après l'arrêt du clic1secondes après
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Après arrêt'));

// sample: 1Échantillonnage chaque seconde
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Périodique'));
```

## 📚 Opérateurs associés

- **[auditTime](./auditTime)** - contrôlé par un temps fixe (version simplifiée de `audit`)
- **[throttle](./throttleTime)** - première valeur émise au début de la période
- **[debounce](./debounceTime)** - émission d'une valeur après stabilisation
- **[sample](./sampleTime)** - échantillonnage au moment d'un autre Observable

## Résumé

L'opérateur `audit` émet la dernière valeur d'une période dynamiquement contrôlée par un Observable personnalisé.

- ✅ Contrôle dynamique de la période possible
- ✅ Échantillonnage adaptatif basé sur la charge
- ✅ Contrôle basé sur d'autres flux
- ⚠️ Un nouvel Observable doit être généré à chaque fois
- ⚠️ Attention à la mémoire en cas d'émission fréquente
