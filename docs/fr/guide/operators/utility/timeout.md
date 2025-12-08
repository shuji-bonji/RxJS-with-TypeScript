---
description: "timeout est un opérateur utilitaire RxJS qui lève une erreur si une valeur n'est pas émise par un Observable dans un délai spécifié. Idéal pour le traitement réactif avec des contraintes de temps telles que le contrôle du délai de requête de l'API, l'attente de la réponse de l'opération de l'utilisateur et la détection du retard du flux. Peut être combiné avec catchError pour mettre en œuvre un comportement de repli, et l'inférence de type TypeScript permet un traitement du délai d'attente sûr."
---

# timeout - Paramétrage du délai d'attente

L'opérateur `timeout` **lance une erreur si une valeur n'est pas émise par l'Observable dans un délai spécifié**.
Il est souvent utilisé dans les traitements réactifs tels que les requêtes API et l'attente de réponses d'opérations utilisateur.


## 🔰 Syntaxe et comportement de base

Si le délai d'attente n'est pas déclenché, il fonctionne normalement ; si un certain temps est dépassé, une erreur se produit.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('response')
  .pipe(
    delay(500), // 👈 S'il est fixé à 1500, il produit `Erreur de timeout : fallback`
    timeout(1000),
    catchError((err) => of('Erreur de timeout : fallback', err))
  )
  .subscribe(console.log);
// Sortie :
// response
```

Dans cet exemple, la valeur est émise après 500ms avec `delay(500)`, ce qui respecte la condition `timeout(1000)`, donc `'response'` est affichée normalement.

Si vous spécifiez `delay(1200)`, une erreur `timeout` est affichée comme suit :
```sh
Erreur de timeout : fallback
TimeoutErrorImpl {stack: 'Error\n at _super (http://localhost:5174/node_mo...s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {...}}
```

[🌐 Documentation officielle RxJS - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Cas d'utilisation typiques

L'exemple suivant montre à la fois un modèle où **un flux retarde et n'émet pas de valeurs, provoquant un dépassement de délai**, et un modèle où **il émet normalement**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback : timeout occurred'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback : timeout triggered'))
  )
  .subscribe(console.log);
// Sortie :
// 0
// 1
// fallback : timeout triggered
// 2
```


## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Zone d'affichage de la sortie
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>Exemple de timeout :</h3>';
document.body.appendChild(timeoutOutput);

// Exemple de réussite du délai d'attente
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Stream normal (pas de timeout) :</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Erreur : ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Valeur de repli après une erreur');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valeur : ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Exemple d'erreur de temporisation
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Stream lent (timeout déclenché) :</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Erreur : ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Valeur de repli après le délai d\'attente');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valeur : ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Résumé

- `timeout` est un opérateur de contrôle qui **lance une erreur s'il n'y a pas d'émission dans un certain délai**
- Efficace pour le traitement des délais d'attente des APIs réseau et des opérations d'interface utilisateur en attente
- Combiné avec `catchError`, **le comportement de repli** peut être spécifié
