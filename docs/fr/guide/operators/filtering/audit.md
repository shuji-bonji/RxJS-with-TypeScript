---
description: "L'opérateur audit est un opérateur de filtrage RxJS qui émet uniquement la dernière valeur pendant une période contrôlée par un Observable personnalisé. Idéal pour un contrôle de timing dynamique."
---

# audit - Dernière valeur sur trigger

L'opérateur `audit` attend qu'un Observable personnalisé émette une valeur, puis émet la **dernière valeur** reçue de la source pendant cette période.
Contrairement à `auditTime` qui contrôle avec un temps fixe, `audit` permet de **contrôler la période avec un Observable dynamique**.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Événements de clic
const clicks$ = fromEvent(document, 'click');

// Diviser les périodes chaque seconde
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Clic enregistré');
});
```

- Quand un clic se produit, une période d'1 seconde commence.
- Seul le dernier clic de cette seconde est émis.
- Après 1 seconde, la période suivante commence.

[🌐 Documentation officielle RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Patterns d'utilisation typiques

- **Échantillonnage à intervalles dynamiques** : Ajuster la période selon la charge
- **Contrôle de timing personnalisé** : Contrôle de période basé sur d'autres Observables
- **Limitation d'événements adaptative** : Réduction selon la situation

## 🔍 Différence avec auditTime

| Opérateur | Contrôle de période | Cas d'utilisation |
|:---|:---|:---|
| `auditTime` | Temps fixe (millisecondes) | Contrôle simple basé sur le temps |
| `audit` | **Observable personnalisé** | **Contrôle de période dynamique** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - 1 seconde fixe
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fixe 1 seconde'));

// audit - période dynamique
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // Période aléatoire 0-2 secondes
    return timer(period);
  })
).subscribe(() => console.log(`Période dynamique: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique selon la charge

Un exemple d'ajustement de l'intervalle d'échantillonnage selon la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// Création de l'UI
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

// Niveau de charge (0: faible, 1: moyen, 2: élevé)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Faible charge', 'Charge moyenne', 'Charge élevée'];
  statusDiv.textContent = `Charge actuelle: ${levels[loadLevel]}`;
});

// Événements de mouvement de souris
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Ajuster la période selon la charge
    const periods = [2000, 1000, 500]; // Faible charge→longue période, Haute charge→courte période
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Position souris: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Afficher maximum 10 entrées
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- En charge faible, réduit à 2 secondes (mode économie d'énergie)
- En charge élevée, échantillonne finement à 500ms
- La période peut être ajustée dynamiquement selon la charge.

## 🎯 Exemple de code pratique 2 : Contrôle de période basé sur un autre flux

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// Création de l'UI
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

// Surveiller la valeur du curseur
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Événements de clic
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Mettre à jour la valeur du curseur
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Contrôler les clics avec audit
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Clic enregistré (intervalle: ${currentInterval}ms)`;
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
// 9  (après 1 seconde, dernière valeur de 0-9)
// 19 (après 2 secondes, dernière valeur de 10-19)
// 29 (après 3 secondes, dernière valeur de 20-29)
```

### 2. L'Observable de durée doit être généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Mauvais exemple: réutiliser la même instance d'Observable
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // Ne fonctionne pas après la première fois
).subscribe();

// ✅ Bon exemple: générer un nouvel Observable à chaque fois
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

Utiliser `audit` sur des flux émettant fréquemment consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Flux rapide (toutes les 10ms)
interval(10).pipe(
  audit(() => timer(1000)) // Échantillonne chaque seconde
).subscribe();
// 100 valeurs s'accumulent en mémoire par seconde, seule la dernière est émise
```

## 🆚 Comparaison avec des opérateurs similaires

| Opérateur | Moment d'émission | Valeur émise | Cas d'utilisation |
|:---|:---|:---|:---|
| `audit` | **Fin de période** | **Dernière** valeur de la période | Obtenir le dernier état de la période |
| `throttle` | **Début de période** | **Première** valeur de la période | Obtenir le premier d'événements consécutifs |
| `debounce` | **Après silence** | Valeur juste avant le silence | Attendre la fin de saisie |
| `sample` | **Quand un autre Observable émet** | Dernière valeur à ce moment | Snapshot périodique |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: dernier clic de la seconde
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: dernier'));

// throttle: premier clic de la seconde
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: premier'));

// debounce: 1 seconde après l'arrêt des clics
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: après arrêt'));

// sample: échantillonne chaque seconde
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: périodique'));
```

## 📚 Opérateurs associés

- **[auditTime](./auditTime)** - Contrôle avec temps fixe (version simplifiée de `audit`)
- **[throttle](./throttleTime)** - Émet la première valeur au début de la période
- **[debounce](./debounceTime)** - Émet la valeur après silence
- **[sample](./sampleTime)** - Échantillonne au timing d'un autre Observable

## Résumé

L'opérateur `audit` émet la dernière valeur d'une période contrôlée dynamiquement par un Observable personnalisé.

- ✅ Contrôle de période dynamique possible
- ✅ Échantillonnage adaptatif selon la charge
- ✅ Contrôle basé sur d'autres flux
- ⚠️ Doit générer un nouvel Observable à chaque fois
- ⚠️ Attention à la mémoire avec des émissions fréquentes
