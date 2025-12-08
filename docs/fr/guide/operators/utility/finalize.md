---
description: "finalize est un opérateur utilitaire RxJS qui définit le traitement qui est toujours exécuté lorsqu'un Observable se termine, génère une erreur ou se désabonne. Idéal pour la libération des ressources, la fin de l'affichage du chargement, le traitement de nettoyage, et d'autres situations où le nettoyage est nécessaire à la fin du flux. Comme try-finally, il garantit une certaine exécution du traitement et aide à prévenir les fuites de mémoire."
---

# finalize - Traitement à la fin

L'opérateur `finalize` définit un traitement qui est **toujours appelé quand un Observable se termine, génère une erreur ou se désabonne**.
Il est idéal pour les traitements de nettoyage et le rejet du chargement de l'interface utilisateur - "les traitements qui doivent toujours être exécutés".

## 🔰 Syntaxe et comportement de base

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Complet')
  .pipe(finalize(() => console.log('Le flux est terminé')))
  .subscribe(console.log);
// Sortie :
// Complet
// Le flux est terminé
```

Dans cet exemple, après avoir émis une valeur avec `of()`, le traitement à l'intérieur de `finalize` est exécuté.
La caractéristique principale est qu'**il est appelé indépendamment de `complete` ou `error`**.

[🌐 Documentation officielle RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Cas d'utilisation typiques

Voici un exemple de commutation de l'affichage du chargement avant et après un flux.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Data')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Chargement démarré');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Chargement terminé');
    })
  )
  .subscribe((value) => console.log('Récupéré:', value));
// Sortie :
// Chargement démarré
// Récupéré: Data
// Chargement terminé
```

## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Zone d'affichage de la sortie
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>Exemple de finalize :</h3>';
document.body.appendChild(finalizeOutput);

// Indicateur de chargement
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Chargement des données...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Affichage de la progression
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Élément pour le message d'achèvement
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Simulation d'extraction de données
interval(500)
  .pipe(
    take(5), // Récupère 5 valeurs
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Traitement de l'élément ${val + 1}...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = 'Traitement terminé !';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'Toutes les données ont été chargées avec succès.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Résumé

- `finalize` est **toujours exécuté indépendamment de l'achèvement, de l'erreur ou de la terminaison manuelle**
- Idéal pour le traitement de nettoyage et la fin de chargement
- Peut être combiné avec d'autres opérateurs (`tap`, `delay`, etc.) pour **sécuriser le nettoyage après un traitement asynchrone**
