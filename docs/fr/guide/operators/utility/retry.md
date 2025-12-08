---
description: "L'opérateur retry réabonne l'Observable source un nombre spécifié de fois lorsqu'une erreur se produit. Il est efficace pour récupérer des échecs de communication temporaires tels que des problèmes de réseau, ou pour un traitement qui peut réussir s'il est réessayé après un échec."
---

# retry - Réessayer en cas d'erreur

L'opérateur `retry` **réabonne l'Observable source un nombre spécifié de fois lorsqu'une erreur se produit**.
Il convient aux **traitements qui peuvent réussir s'ils sont réessayés**, tels que les pannes de réseau temporaires.

## 🔰 Syntaxe et comportement de base

### retry(count) - Forme de base

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Erreur temporaire'))
  .pipe(
    retry(2), // Réessaie jusqu'à 2 fois
    catchError((error) => of(`Erreur finale : ${error.message}`))
  )
  .subscribe(console.log);
// Sortie :
// Erreur finale : Erreur temporaire
```

Dans cet exemple, après le premier échec, il y a jusqu'à deux tentatives, et si toutes échouent, un message de repli est affiché.

### retry(config) - Format de l'objet de configuration (RxJS 7.4+)

Depuis RxJS 7.4, vous pouvez passer un objet de configuration pour un contrôle plus détaillé.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Erreur temporaire'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Tentative ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Réessaie jusqu'à 2 fois
      delay: 1000,        // Attend 1 seconde avant de réessayer (utilise asyncScheduler en interne)
      resetOnSuccess: true // Réinitialise le compte en cas de succès
    }),
    catchError((error) => of(`Erreur finale : ${error.message}`))
  )
  .subscribe(console.log);

// Sortie :
// Tentative 1
// Tentative 2
// Tentative 3
// Erreur finale : Erreur temporaire
```

> [!NOTE] Contrôle du délai de réessai
> Lorsque l'option `delay` est spécifiée, **asyncScheduler** est utilisé en interne. Pour plus de détails sur le contrôle de la temporisation des tentatives (comme le backoff exponentiel), voir [Types de schedulers et utilisation - Contrôle du réessai d'erreur](/fr/guide/schedulers/types#contrôle-du-réessai-derreur).

[🌐 Documentation officielle RxJS - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Cas d'utilisation typiques

L'exemple suivant est une configuration qui réessaie jusqu'à 3 fois pour **un traitement asynchrone qui réussit ou échoue de manière aléatoire**.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Échec #${attempt}`));
      } else {
        return of(`Succès #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Échec final : ${err.message}`))
  )
  .subscribe(console.log);
// Sortie :
// Succès #1
// Succès #5
// Succès #6
// Échec final : Échec #7
```

## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Zone d'affichage de la sortie
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>Exemple de retry (simulation de requête API) :</h3>';
document.body.appendChild(retryOutput);

// Affichage de l'état de la demande
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// Requête API qui réussit ou échoue de manière aléatoire
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Tentative #${attemptCount} Envoi de la requête...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Tentative #${attemptCount} Échec : Erreur de réseau`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Erreur de réseau'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Tentative #${attemptCount} Succès !`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Données récupérées avec succès' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `Tous les essais ont échoué : ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Les réessais ont échoué' });
    })
  );
}

// Bouton de démarrage de requête
const startButton = document.createElement('button');
startButton.textContent = 'Démarrer la requête';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Résultat final : ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Résultat final : ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Résumé

- `retry(n)` réessaie jusqu'à `n` fois lorsqu'un Observable émet une erreur
- `retry` **ré-exécute jusqu'à ce que l'exécution soit réussie** (une erreur est émise si les échecs se poursuivent)
- Efficace pour les **APIs asynchrones et les requêtes réseau** où des échecs temporaires se produisent
- Typiquement combiné avec `catchError` pour spécifier un **traitement de secours**
- Depuis RxJS 7.4+, vous pouvez spécifier `delay`, `resetOnSuccess`, etc. en utilisant le format d'objet de configuration

## Pages connexes

- [retry et catchError](/fr/guide/error-handling/retry-catch) - Modèles de combinaison de retry et catchError, exemples d'utilisation pratique
- [Débogage des tentatives](/fr/guide/error-handling/retry-catch#débogage-des-tentatives) - Méthodes de suivi du nombre de tentatives (5 modèles d'implémentation)
- [Types de schedulers et utilisation](/fr/guide/schedulers/types#contrôle-du-réessai-derreur) - Contrôle détaillé du timing des tentatives, implémentation d'un backoff exponentiel
- [Techniques de débogage RxJS](/fr/guide/debugging/#scénario-6-suivi-des-tentatives-de-réessai) - Scénarios de débogage des tentatives de réessai
