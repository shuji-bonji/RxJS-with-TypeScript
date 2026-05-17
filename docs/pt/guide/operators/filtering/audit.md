---
description: "O operador `audit` é um operador de filtragem RxJS que emite apenas o último valor dentro do período controlado por um Observable personalizado. Ideal para controle dinâmico de timing."
head:
  - - meta
    - name: keywords
      content: RxJS, audit, operador de filtragem, controle dinâmico de timing, Observable, TypeScript
---

# audit - dernière valeur de la période de contrôle émise

L'opérateur `audit` attend qu'un Observable personnalisé émette une valeur et émet la **dernière valeur** émise par la source au cours de cette période.
Alors que `auditTime` est contrôlé par un temps fixe, `audit` permet de **contrôler la période** avec un Observable dynamique.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento de clique
const clicks$ = fromEvent(document, 'click');

// 1Delimitar período a cada N segundos
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Clique registrado');
});
```

- Lorsqu'un clic se produit, une période d'une seconde commence.
- Seul le dernier clic de cette période d'une seconde est émis.
- Après une seconde, la période suivante commence.

[🌐 Documentation officielle de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet de désabonner `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/pt/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Modèles d'utilisation typiques

- **Échantillonnage dynamique par intervalles** : ajuster la durée en fonction de la charge
- **Contrôle de timing personnalisé** : contrôle de la période basé sur d'autres Observables
- **Limitation adaptative des événements** : régulation sensible au contexte

## 🔍 Différences avec auditTime

| Operador | Controle de período | Caso de uso |
|---|---|---|
| `auditTime` | Tempo fixo (milissegundos) | Controle simples baseado em tempo |
| `audit` | **Observable personalizado** | **Controle dinâmico de período** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fixo1segundos
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fixo1segundos'));

// audit - Período dinâmico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~2Período aleatório de segundos
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 Exemple de code pratique 1 : Échantillonnage dynamique basé sur la charge

Voici un exemple d'ajustement de l'intervalle d'échantillonnage en fonction de la charge du système.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICriar
const output = document.createElement('div');
output.innerHTML = '<h3>Amostragem dinâmica</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Mudar carga';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Nível de carga (0: Baixa,1: Média,2: Alta)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carga baixa', 'Carga média', 'Carga alta'];
  statusDiv.textContent = `Carga atual: ${levels[loadLevel]}`;
});

// Evento mousemove
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Ajustar período conforme carga
    const periods = [2000, 1000, 500]; // Carga baixa→Período longo, carga alta→Período curto
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posição do mouse: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Máx.10itens exibidos
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

// UICriar
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervalo: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Monitorar valor do slider
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Evento de clique
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Atualizar valor do slider
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Cliqueauditcontrolado por
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Registro de clique (intervalo: ${currentInterval}ms)`;
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
// Saída:
// 9  (1segundos depois,0~9último valor de)
// 19 (2segundos depois,10~19último valor de)
// 29 (3segundos depois,20~29último valor de)
```

### 2. L'Observable de durée est généré à chaque fois

La fonction passée à `audit` **doit retourner un nouvel Observable à chaque fois**.

```ts
// ❌ Exemplo ruim: MesmaObservableinstância reutilizada
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2não funciona após a 2ª vez
).subscribe();

// ✅ Bom exemplo: Novo a cada vezObservableGera
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Mémoire et performance

L'utilisation de `audit` sur des flux où des valeurs sont fréquemment émises consomme de la mémoire.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Stream rápido (10ms a cada)
interval(10).pipe(
  audit(() => timer(1000)) // 1Amostragem a cada segundo
).subscribe();
// 1Em N segundos,100valores se acumulam na memória e apenas o último1é emitido
```

## 🆚 Comparaison avec des opérateurs similaires

| Operador | Quando emitir | Valor emitido | Caso de uso |
|---|---|---|---|
| `audit` | No **fim** do período | O **último** valor do período | Obter o último estado do período |
| `throttle` | No **início** do período | O **primeiro** valor do período | Obter o início de uma sequência de eventos |
| `debounce` | **Após estabilização** | Valor logo antes da estabilização | Aguardar conclusão da entrada |
| `sample` | **Quando outro Observable dispara** | Valor mais recente naquele momento | Snapshot periódico |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Último clique em N segundos
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Último'));

// throttle: 1Primeiro clique em N segundos
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Primeiro'));

// debounce: Após parar clique1segundos depois
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Após parada'));

// sample: 1Amostragem a cada segundo
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periódico'));
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
