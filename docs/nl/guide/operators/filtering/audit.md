---
description: "De `audit`-operator is een RxJS-filteroperator die alleen de laatste waarde emitteert binnen de periode die wordt gecontroleerd door een aangepaste Observable. Ideaal voor dynamische timing-controle."
head:
  - - meta
    - name: keywords
      content: RxJS, audit, filteroperator, dynamische timing-controle, Observable, TypeScript
---

# audit - dernière valeur de la période de contrôle émise

L'opérateur `audit` attend qu'un Observable personnalisé émette une valeur et émet la **dernière valeur** émise par la source au cours de cette période.
Alors que `auditTime` est contrôlé par un temps fixe, `audit` permet de **contrôler la période** avec un Observable dynamique.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klik-event
const clicks$ = fromEvent(document, 'click');

// 1Periode elke N seconden afbakenen
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klik geregistreerd');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/nl/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge
- **Contrôle de timing personnalisé** : contrôle de la période basé sur d'autres Observables
- **Limitation adaptative des événements** : régulation sensible au contexte

## 🔍 Différences avec auditTime

| Operator | Periode-controle | Gebruikscase |
|---|---|---|
| `auditTime` | Vaste tijd (milliseconden) | Eenvoudige tijd-gebaseerde controle |
| `audit` | **Aangepaste Observable** | **Dynamische periode-controle** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Vast1seconden
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Vast1seconden'));

// audit - Dynamische periode
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~2Willekeurige periode van seconden
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UIMaken
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamische bemonstering</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Belasting wijzigen';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Belastingsniveau (0: Laag,1: Gemiddeld,2: Hoog)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Lage belasting', 'Gemiddelde belasting', 'Hoge belasting'];
  statusDiv.textContent = `Huidige belasting: ${levels[loadLevel]}`;
});

// Mousemove-event
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Periode aanpassen aan belasting
    const periods = [2000, 1000, 500]; // Lage belasting→Lange periode, hoge belasting→Korte periode
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Muispositie: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10items getoond
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

// UIMaken
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Interval: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Schuifregelaarwaarde monitoren
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Klik-event
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Schuifregelaarwaarde bijwerken
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Klikauditgestuurd door
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Klik-opname (interval: ${currentInterval}ms)`;
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
// Uitvoer:
// 9  (1seconden later,0~9laatste waarde van)
// 19 (2seconden later,10~19laatste waarde van)
// 29 (3seconden later,20~29laatste waarde van)
```

### 2. L'Observable de durée est généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Slecht voorbeeld: ZelfdeObservableinstantie hergebruikt
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2werkt niet na de 2e keer
).subscribe();

// ✅ Goed voorbeeld: Elke keer nieuwObservableGenereert
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

L'utilisation de `audit` sur des flux où des valeurs sont fréquemment émises consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Snelle stream (10ms elke)
interval(10).pipe(
  audit(() => timer(1000)) // 1Bemonstering elke seconde
).subscribe();
// 1In N seconden,100waarden hopen op in geheugen en alleen de laatste1wordt geëmitteerd
```

## 🆚 Comparaison avec des opérateurs similaires

| Operator | Wanneer emitteren | Geëmitteerde waarde | Gebruikscase |
|---|---|---|---|
| `audit` | Aan het **eind** van de periode | De **laatste** waarde van de periode | Laatste status binnen periode verkrijgen |
| `throttle` | Aan het **begin** van de periode | De **eerste** waarde van de periode | Begin van een gebeurtenisreeks verkrijgen |
| `debounce` | **Na stabilisatie** | Waarde net voor stabilisatie | Wachten op invoervoltooiing |
| `sample` | **Wanneer een andere Observable triggert** | Meest recente waarde op dat moment | Periodieke snapshot |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Laatste klik in N seconden
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Laatste'));

// throttle: 1Eerste klik in N seconden
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Eerste'));

// debounce: Na klik-stop1seconden later
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Na stop'));

// sample: 1Bemonstering elke seconde
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periodiek'));
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
