---
description: "L'opérateur d'audit est un opérateur de filtrage RxJS qui n'émet que la dernière valeur au cours de la période contrôlée par l'observable personnalisé. Il est idéal pour le contrôle dynamique des délais."
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

// 1Périodes de temps séparées toutes les secondes
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Le clic a été enregistré');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Attention en code de production

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge.
- **Custom timing control** : contrôle de la période basé sur d'autres observables.
- Limitation adaptative des événements** : amincissement sensible au contexte.

## 🔍 Différences avec auditTime

| Opérateur. | Contrôle des périodes | Cas d'utilisation. |
|---|---|---|
| `auditTime`. | Temps fixe (millisecondes) | Contrôle simple basé sur le temps |
| `audit`. | **Observable personnalisé** | **Contrôle dynamique de la période**. |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fixe1fixes
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fixe1fixes'));

// audit - Période dynamique
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~.2Période aléatoire de secondes
    return timer(period);
  })
).subscribe(() => console.log(`Période dynamique: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICréation
const output = document.createElement('div');
output.innerHTML = '<h3>Échantillonnage dynamique</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Changement de charge';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Niveau de charge (0: Faible charge,1: Charge moyenne,2: Charge élevée)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Faible charge', 'Charge moyenne', 'Charge élevée'];
  statusDiv.textContent = `Charge actuelle: ${levels[loadLevel]}`;
});

// Mouvement de la souris
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Durée en fonction de la charge
    const periods = [2000, 1000, 500]; // Faible charge→Longue durée, charge élevée→Courte durée
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Position de la souris: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10Affichage jusqu'à
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Échantillonnage réduit à des intervalles de 2 s lorsque la charge est faible (mode d'économie d'énergie).
- Échantillonnage fin à intervalles de 500 ms lorsque la charge est élevée.
- La période peut être ajustée dynamiquement en fonction de la charge.

## 🎯 Exemple de code pratique 2 : Contrôle de la période basé sur d'autres flux

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UICréation
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

// Contrôler les valeurs du curseur
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

// Mise à jour des valeurs du curseur
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Cliquer surauditContrôlé par
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Enregistrement du clic (intervalle: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Notes.

### 1. la première valeur n'est pas immédiatement émise

Après que le `audit` a reçu la première valeur, il attend la fin de la période.

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
// 9  (1Quelques secondes plus tard,0~.9Dernière valeur de)
// 19 (2Quelques secondes plus tard,10~.19Dernière valeur de)
// 29 (3Quelques secondes plus tard,20~.29Dernière valeur de)
```

### L'observable duration est généré à chaque fois.

Les fonctions passées à `audit` **doivent retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Mauvais exemple: Si la même instanceObservableest utilisée et réutilisée
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2Ne fonctionne pas après la deuxième fois
).subscribe();

// ✅ Bon exemple: Nouvelle instance créée à chaque foisObservableGénérer un
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. mémoire et performance

L'utilisation de `audit` sur les flux où des valeurs sont émises consomme fréquemment de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// flux rapide (10mspar seconde)
interval(10).pipe(
  audit(() => timer(1000)) // 1Échantillonnage toutes les secondes
).subscribe();
// 1par seconde100Les valeurs sont stockées en mémoire et seule la dernière est émise.1Seule la dernière est émise
```

## 🆚 Comparaison avec des opérateurs similaires

| Opérateurs | Quand délivrer | Valeur à délivrer | Cas d'utilisation. |
|---|---|---|---|
| `audit`. | A la **fin** de la période | La **dernière** valeur de la période | Obtenir le dernier statut dans la période |
| `throttle`. | Au **début** de la période | Valeur du **premier** de la période | Obtenir le début d'une séquence d'événements. |
| `debounce`. | **Après** stationnaire**. | Valeur juste avant la stationnaire | Attendre la fin d'une entrée |
| `sample`. | **Quand un autre Observable se déclenche**. | Valeur la plus récente à ce moment-là | Instantané périodique |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Dernier clic en secondes
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Dernier clic'));

// throttle: 1Premier clic en secondes
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Premier'));

// debounce: Après l'arrêt du clic1Secondes après
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Après l'arrêt'));

// sample: 1Échantillonnage toutes les secondes
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Périodique'));
```

## 📚 Opérateurs associés.

- **[auditTime](. /auditTime)** - contrôlé par une heure fixe (version simplifiée de `audit`).
- **[throttle](. /throttleTime)** - première valeur émise au début de la période.
- **[debounce](. /debounceTime)** - émission d'une valeur après une période d'inactivité.
- **[sample](. /sampleTime)** - échantillonnage au moment où un autre observable est utilisé.

## Résumé.

L'opérateur `audit` émet la dernière valeur à l'intérieur d'une période dynamiquement contrôlée par un Observable personnalisé.

- ✅ Le contrôle dynamique de la période est possible.
- ✅ Échantillonnage adaptatif basé sur la charge
- ✅ Contrôle basé sur d'autres flux
- ⚠️ Un nouvel observable doit être généré à chaque fois.
- ⚠️ Sensible à la mémoire en cas d'émission fréquente
