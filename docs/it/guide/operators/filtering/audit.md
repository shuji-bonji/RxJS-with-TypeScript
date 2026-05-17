---
description: "L'operatore `audit` è un operatore di filtraggio RxJS che emette solo l'ultimo valore all'interno del periodo controllato da un Observable personalizzato. Ideale per il controllo dinamico del timing."
head:
  - - meta
    - name: keywords
      content: RxJS, audit, operatore di filtraggio, controllo dinamico del timing, Observable, TypeScript
---

# audit - dernière valeur de la période de contrôle émise

L'opérateur `audit` attend qu'un Observable personnalisé émette une valeur et émet la **dernière valeur** émise par la source au cours de cette période.
Alors que `auditTime` est contrôlé par un temps fixe, `audit` permet de **contrôler la période** avec un Observable dynamique.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento click
const clicks$ = fromEvent(document, 'click');

// 1Delimita il periodo ogni N secondi
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Click registrato');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/it/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge
- **Contrôle de timing personnalisé** : contrôle de la période basé sur d'autres Observables
- **Limitation adaptative des événements** : régulation sensible au contexte

## 🔍 Différences avec auditTime

| Operatore | Controllo del periodo | Caso d'uso |
|---|---|---|
| `auditTime` | Tempo fisso (millisecondi) | Controllo basato sul tempo semplice |
| `audit` | **Observable personalizzato** | **Controllo dinamico del periodo** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fisso1secondi
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fisso1secondi'));

// audit - Periodo dinamico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~2Periodo casuale di secondi
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICreare
const output = document.createElement('div');
output.innerHTML = '<h3>Campionamento dinamico</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Cambia carico';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Livello di carico (0: Basso,1: Medio,2: Alto)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carico basso', 'Carico medio', 'Carico alto'];
  statusDiv.textContent = `Carico attuale: ${levels[loadLevel]}`;
});

// Evento mousemove
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Regolare il periodo in base al carico
    const periods = [2000, 1000, 500]; // Carico basso→Periodo lungo, carico alto→Periodo breve
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posizione del mouse: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10elementi visualizzati
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

// UICreare
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervallo: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Monitora valore slider
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Evento click
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Aggiorna valore slider
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Clickauditcontrollato da
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Registrazione click (intervallo: ${currentInterval}ms)`;
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
// Output:
// 9  (1secondi dopo,0~9ultimo valore di)
// 19 (2secondi dopo,10~19ultimo valore di)
// 29 (3secondi dopo,20~29ultimo valore di)
```

### 2. L'Observable de durée est généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Esempio negativo: StessaObservableistanza riutilizzata
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2non funziona dopo la 2a volta
).subscribe();

// ✅ Esempio positivo: Nuovo ogni voltaObservableGenera
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

L'utilisation de `audit` sur des flux où des valeurs sont fréquemment émises consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Stream veloce (10ms ogni)
interval(10).pipe(
  audit(() => timer(1000)) // 1Campionamento ogni secondo
).subscribe();
// 1In N secondi,100valori si accumulano in memoria e solo l'ultimo1viene emesso
```

## 🆚 Comparaison avec des opérateurs similaires

| Operatore | Quando emettere | Valore emesso | Caso d'uso |
|---|---|---|---|
| `audit` | Alla **fine** del periodo | L'**ultimo** valore del periodo | Ottenere l'ultimo stato del periodo |
| `throttle` | All'**inizio** del periodo | Il **primo** valore del periodo | Ottenere l'inizio di una sequenza di eventi |
| `debounce` | **Dopo la stabilizzazione** | Valore appena prima della stabilizzazione | Attendere il completamento dell'input |
| `sample` | **Quando un altro Observable si attiva** | Valore più recente in quel momento | Snapshot periodico |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Ultimo click in N secondi
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Ultimo'));

// throttle: 1Primo click in N secondi
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Primo'));

// debounce: Dopo l'arresto del click1secondi dopo
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Dopo l'arresto'));

// sample: 1Campionamento ogni secondo
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periodico'));
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
